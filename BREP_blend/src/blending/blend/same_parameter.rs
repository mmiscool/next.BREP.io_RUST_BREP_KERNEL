//! `BREP_BLEND_SAME_PARAMETER` (on unless `=0`/`off`; promoted 2026-11-19): a blend's iso-line trims made to
//! share their edges' parameter.
//!
//! A corner patch, a canal corner or a revolved sector meets the edge it
//! shares with a stripe along one of its own iso rows, and the trim on it is
//! that row's straight parameter line: exact in geometry (the row IS the
//! edge's curve, to 1e-14), but traversed at the row's own pace. The edge
//! runs at the stripe's pace (a one-conic section arc, a straight rung), so
//! the pcurve and the edge part company at the same parameter. Measured
//! 2026-11-19 on the blend/fillet/chamfer lib, shipped bodies: matched gaps
//! up to 0.2 against geometric gaps of 1e-14..4e-8 (a canal corner's 2/3
//! conic edge, a wrap corner's straight edge, a mixed corner's revolved
//! sector). `validate` reads matched gaps, and so does any STEP reader that
//! pairs an edge's parameter with its pcurve's.
//!
//! The rebuild keeps the iso coordinate EXACTLY (the pcurve's image stays on
//! the row, so the face's geometry does not move) and replaces only the map
//! along the row: the row parameter `s(f)` at the edge's own traversal
//! fraction `f`, read by projecting the edge onto the row, with the exact
//! derivative `ds/df` from the two tangents, as piecewise-cubic Hermite
//! segments pinned to the old pcurve's ends (so the loop's uv wire, and any
//! seam it straddles, are as before). Stations double until the matched gap
//! reads at most `max(TARGET_REL x diagonal, 2 x the old trim's own geometric
//! gap)`; the rebuild is kept only if it reads below the old matched gap and
//! its geometric gap is no larger. Exact in geometry (the trim stays on the
//! row, monotone, between the same ends); the parameter is exact at the
//! stations and to the target between them.
//!
//! A row that stops (no speed) at a trim end, as a canal or wrap corner's
//! rows do where the ball pivots about a contact that does not move along the
//! edge, makes `s(f)` go as the square root of the fraction there, which even
//! stations in the fraction cannot follow. Those trims are stationed evenly
//! along the ROW instead, each row point's fraction read back off the edge (a
//! regular curve): the fractions crowd where the row stops, and the gap is
//! read in every cell. The patch cannot be laid out in the edge's parameter
//! instead: its other rows would then go as that square root.
//!
//! Reach, measured 2026-11-19: a revolved sector's rim (the open mixed
//! corner), a stripe's end row, the one-conic switch's off arm, and the canal
//! and wrap corners' rows that stop at their stops rebuild. Fitted, non-iso
//! corner trims (a crease) are left to their builders. A trim fitted ALONG a
//! row (every control on it, as the edge-preserving blend's kept rim is
//! interpolated at its stations) is an iso line of higher degree and
//! rebuilds the same way.

use crate::topology::BrepSolid;
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};
use std::collections::HashSet;

/// The matched gap a rebuilt trim must reach, relative to the model diagonal
/// (floor 1e-12): 100x under the kernel-built bar the case gate holds shipped
/// bodies to (1e-4). A corner patch's row can stop (no speed), where the map from
/// the edge's parameter has an unbounded slope that no polynomial pcurve meets
/// exactly; the rebuilt trim stays ON the row whatever the target, so the
/// target bounds only the parameter, never the geometry.
pub const TARGET_REL: f64 = 1e-6;

/// A matched gap at or under this (relative to the diagonal) is left alone.
const TRIGGER_REL: f64 = 1e-9;

pub(crate) fn same_parameter_on() -> bool {
    crate::switch!(live "BREP_BLEND_SAME_PARAMETER", true)
}

/// Rebuild the iso-line trims on the faces this operation touched (a face
/// with a coedge on an edge `input` does not have) whose matched gap is above
/// the trigger. Returns how many trims were rebuilt. No-op with the switch
/// off.
pub fn same_parameter_trims(input: &BrepSolid, result: &mut BrepSolid) -> usize {
    if !same_parameter_on() {
        return 0;
    }
    let existing: HashSet<u64> = input.edges.iter().map(|edge| edge.id).collect();
    // The true extent: a relative target stays relative on a sub-unit part.
    let diagonal = vertex_diagonal(result);
    if !(diagonal > 0.0) {
        return 0;
    }
    let (trigger, target) = (TRIGGER_REL * diagonal, (TARGET_REL * diagonal).max(1e-12));
    let edges: std::collections::HashMap<u64, (NurbsCurve, f64, f64, bool)> = result
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.curve.clone(), edge.t0, edge.t1, edge.degenerate)))
        .collect();
    let mut rebuilt = 0;
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            let touched = face.loops.iter().flat_map(|l| &l.coedges).any(|c| !existing.contains(&c.edge_id));
            if !touched {
                continue;
            }
            // An edge used twice in one face (a seam) keeps both trims.
            let mut uses: std::collections::HashMap<u64, usize> = Default::default();
            for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
                *uses.entry(coedge.edge_id).or_default() += 1;
            }
            let surface = face.surface.clone();
            for coedge in face.loops.iter_mut().flat_map(|l| l.coedges.iter_mut()) {
                if uses.get(&coedge.edge_id).copied().unwrap_or(0) != 1 {
                    continue;
                }
                let Some((curve, t0, t1, degenerate)) = edges.get(&coedge.edge_id) else { continue };
                if *degenerate {
                    continue;
                }
                // Read at two counts with no common station but the ends: a
                // trim interpolated at even fractions (the kept rim of an
                // edge-preserving blend, at its march's 64) meets the edge at
                // exactly the stations one count reads, and parts from it
                // only between them.
                let Some(old) = matched_gap(&surface, &coedge.pcurve, curve, *t0, *t1, coedge.forward, 64) else { continue };
                let Some(old) = matched_gap(&surface, &coedge.pcurve, curve, *t0, *t1, coedge.forward, 97).map(|odd| old.max(odd)) else { continue };
                if old <= trigger || iso_line(&coedge.pcurve).is_none() {
                    continue;
                }
                // The old trim lies on its row: its distance from the edge, as
                // point sets, is the floor no rebuild can read below, and the
                // geometry the rebuild must keep.
                let Some(old_geometric) = geometric_gap(&surface, &coedge.pcurve, curve, *t0, *t1) else { continue };
                let owed = target.max(2.0 * old_geometric);
                let verified = iso_same_parameter_why(&surface, &coedge.pcurve, curve, *t0, *t1, coedge.forward, owed).and_then(|pcurve| {
                    let matched = matched_gap(&surface, &pcurve, curve, *t0, *t1, coedge.forward, 512).ok_or("gap read")?;
                    let geometric = geometric_gap(&surface, &pcurve, curve, *t0, *t1).ok_or("geometric read")?;
                    if !(matched < old) {
                        return Err(format!("no better: {matched:.3e} against {old:.3e}"));
                    }
                    if geometric > old_geometric + 1e-10 * diagonal {
                        return Err(format!("trim moved: geometric {geometric:.3e} against {old_geometric:.3e}"));
                    }
                    Ok(pcurve)
                });
                // A trim that cannot be rebuilt to the target keeps its old
                // pcurve.
                if let Ok(pcurve) = verified {
                    coedge.pcurve = pcurve;
                    rebuilt += 1;
                }
            }
        }
    }
    rebuilt
}

/// The iso line's fixed coordinate and its row: `(fixed_is_v, fixed, s_start, s_end)`.
/// A clamped polynomial pcurve whose controls all share one coordinate lies
/// on that row whatever its degree (each point is a convex combination of the
/// controls), so a fitted trim along a row (the edge-preserving blend's kept
/// rim, interpolated at its stations) is an iso line as much as a straight
/// one; its ends are its first and last controls.
fn iso_line(pcurve: &NurbsCurve) -> Option<(bool, f64, f64, f64)> {
    let (degree, controls) = (pcurve.degree, &pcurve.control_points);
    if degree < 1 || controls.len() < 2 || pcurve.knots.len() != controls.len() + degree + 1 {
        return None;
    }
    if controls.iter().any(|c| (c.w - 1.0).abs() > 1e-15) {
        return None;
    }
    let knots = &pcurve.knots;
    let clamped = knots[..=degree].iter().all(|k| *k == knots[0])
        && knots[knots.len() - 1 - degree..].iter().all(|k| *k == knots[knots.len() - 1]);
    if !clamped {
        return None;
    }
    let (a, b) = (controls[0], controls[controls.len() - 1]);
    // Iso to the last bits a builder's arithmetic leaves (1e-12 of the
    // coordinate); the row is read at the start point's value.
    let same = |p: f64, q: f64| (p - q).abs() <= 1e-12 * (1.0 + p.abs().max(q.abs()));
    if controls.iter().all(|c| same(a.y, c.y)) && !same(a.x, b.x) {
        Some((true, a.y, a.x, b.x))
    } else if controls.iter().all(|c| same(a.x, c.x)) && !same(a.y, b.y) {
        Some((false, a.x, a.y, b.y))
    } else {
        None
    }
}

/// The rebuilt trim for an iso-line pcurve, or `None` when it is not an iso
/// line, leaves its surface's domain, or cannot reach `target`.
pub fn iso_same_parameter(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    edge: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
    target: f64,
) -> Option<NurbsCurve> {
    iso_same_parameter_why(surface, pcurve, edge, t0, t1, forward, target).ok()
}

/// [`iso_same_parameter`] with the reason it declines.
pub fn iso_same_parameter_why(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    edge: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
    target: f64,
) -> Result<NurbsCurve, String> {
    let (fixed_is_v, fixed, s_start, s_end) = iso_line(pcurve).ok_or("not an iso line")?;
    let [u_lo, u_hi] = surface.domain_u()?;
    let [v_lo, v_hi] = surface.domain_v()?;
    let (row, (lo, hi), (fixed_lo, fixed_hi)) = if fixed_is_v {
        (surface.iso_curve_v(fixed)?, (u_lo, u_hi), (v_lo, v_hi))
    } else {
        (surface.iso_curve_u(fixed)?, (v_lo, v_hi), (u_lo, u_hi))
    };
    if !(fixed >= fixed_lo && fixed <= fixed_hi && s_start.min(s_end) >= lo && s_start.max(s_end) <= hi) {
        return Err(format!("outside the domain: fixed {fixed} in {fixed_lo}..{fixed_hi}, row {s_start}..{s_end} in {lo}..{hi}"));
    }
    let edge_at = |f: f64| if forward { t0 + (t1 - t0) * f } else { t1 - (t1 - t0) * f };
    let dt_df = if forward { t1 - t0 } else { t0 - t1 };
    let (s_lo, s_hi) = (s_start.min(s_end), s_start.max(s_end));
    let mut stations = 16usize;
    let mut last = f64::NAN;
    let mut refused: Option<usize> = None;
    let rising = s_end > s_start;
    while stations <= 1024 {
        let mut values = Vec::with_capacity(stations + 1);
        let mut slopes: Vec<Option<f64>> = Vec::with_capacity(stations + 1);
        for k in 0..=stations {
            let f = k as f64 / stations as f64;
            let t = edge_at(f);
            let target_point = edge.evaluate(t)?;
            let guess = s_start + (s_end - s_start) * f;
            let s = if k == 0 {
                s_start
            } else if k == stations {
                s_end
            } else {
                project_on_row(&row, target_point, guess, s_lo, s_hi).ok_or("projection")?
            };
            let row_d = row.derivatives(s, 1)?;
            let edge_d = edge.derivatives(t, 1)?;
            let (row_tangent, edge_tangent) = (*row_d.get(1).ok_or("row derivative")?, *edge_d.get(1).ok_or("edge derivative")?);
            let speed = row_tangent.dot(row_tangent);
            values.push(s);
            // A row that stops (no speed) has no exact slope there; it is
            // read off the neighbours below.
            slopes.push((speed > 1e-24 * (1.0 + edge_tangent.dot(edge_tangent))).then(|| edge_tangent.dot(row_tangent) / speed * dt_df));
        }
        // Monotone along the row, as the old line was: where the row barely
        // moves in space, the projection's last digits may step back, which
        // moves no point; hold the value instead.
        for k in 1..values.len() {
            values[k] = if rising { values[k].max(values[k - 1]) } else { values[k].min(values[k - 1]) };
        }
        let h = 1.0 / stations as f64;
        let mut slopes: Vec<f64> = (0..values.len())
            .map(|k| {
                let central = {
                    let (a, b) = (k.saturating_sub(1), (k + 1).min(values.len() - 1));
                    (values[b] - values[a]) / ((b - a) as f64 * h)
                };
                let slope = slopes[k].unwrap_or(central);
                if (rising && slope < 0.0) || (!rising && slope > 0.0) { 0.0 } else { slope }
            })
            .collect();
        // Fritsch-Carlson: a segment's end slopes are limited against its
        // secant so the cubic between them is monotone (a flat segment takes
        // flat ends). Positive slopes alone do not keep it monotone.
        for k in 0..values.len() - 1 {
            let secant = (values[k + 1] - values[k]) / h;
            if secant == 0.0 {
                slopes[k] = 0.0;
                slopes[k + 1] = 0.0;
                continue;
            }
            let (alpha, beta) = (slopes[k] / secant, slopes[k + 1] / secant);
            let norm = alpha * alpha + beta * beta;
            if norm > 9.0 {
                let tau = 3.0 / norm.sqrt();
                slopes[k] = tau * alpha * secant;
                slopes[k + 1] = tau * beta * secant;
            }
        }
        let candidate = hermite_pcurve(&values, &slopes, fixed_is_v, fixed).ok_or("hermite")?;
        // A monotone control polygon inside the row's segment bounds the
        // curve to it, monotone: the trim covers the old row segment exactly.
        let along: Vec<f64> = candidate.control_points.iter().map(|c| if fixed_is_v { c.x } else { c.y }).collect();
        let inside = along.iter().all(|value| *value >= s_lo && *value <= s_hi);
        let ordered = along.windows(2).all(|w| if rising { w[1] >= w[0] } else { w[1] <= w[0] });
        // The circle limit keeps each cubic monotone but not its polygon
        // ordered (that needs alpha + beta <= 3); finer stations shrink the
        // slope ratios, so a refused polygon tries the next count.
        if !(inside && ordered) {
            refused = Some(stations);
            stations *= 2;
            continue;
        }
        let gap = matched_gap(surface, &candidate, edge, t0, t1, forward, 4 * stations).ok_or("gap read")?;
        last = gap;
        if gap <= target {
            return Ok(candidate);
        }
        stations *= 2;
    }
    let by_edge = match refused {
        Some(at) if last.is_nan() => format!("control polygon leaves the row segment or backtracks up to {at} stations"),
        _ => format!("target {target:.3e} not reached at 1024 stations: {last:.3e} (last polygon refused at {refused:?} stations)"),
    };
    // A row that stops (no speed) at a trim end: the row parameter goes as
    // the square root of the edge's there, and stations even in the edge's
    // fraction miss it. Station the ROW evenly instead and read each row
    // point's fraction off the edge (a regular curve, so that projection is
    // well conditioned): the fractions crowd where the row stops, as the map
    // needs.
    let row_stationed = || -> Result<NurbsCurve, String> {
        let (t_lo, t_hi) = (t0.min(t1), t0.max(t1));
        let fraction_of = |t: f64| if forward { (t - t0) / (t1 - t0) } else { (t1 - t) / (t1 - t0) };
        let mut last = f64::NAN;
        let mut stations = 16usize;
        while stations <= 1024 {
            let (mut fractions, mut values, mut slopes) = (vec![0.0], vec![s_start], vec![f64::NAN]);
            let mut t = edge_at(0.0);
            for k in 1..stations {
                let s = s_start + (s_end - s_start) * k as f64 / stations as f64;
                let point = row.evaluate(s)?;
                t = project_on_row(edge, point, t, t_lo, t_hi).ok_or("edge projection")?;
                let f = fraction_of(t);
                if !(f > *fractions.last().unwrap() + 1e-14 && f < 1.0 - 1e-14) {
                    continue;
                }
                let row_tangent = *row.derivatives(s, 1)?.get(1).ok_or("row derivative")?;
                let edge_tangent = *edge.derivatives(t, 1)?.get(1).ok_or("edge derivative")?;
                let df_ds = row_tangent.dot(edge_tangent) / edge_tangent.dot(edge_tangent) / dt_df;
                fractions.push(f);
                values.push(s);
                slopes.push(if df_ds.abs() > 1e-300 { 1.0 / df_ds } else { f64::NAN });
            }
            fractions.push(1.0);
            values.push(s_end);
            slopes.push(f64::NAN);
            // The row's end slopes, and any the row's stop leaves unread,
            // from the neighbouring secants; Fritsch-Carlson limits all.
            let n = values.len();
            let secant = |k: usize| (values[k + 1] - values[k]) / (fractions[k + 1] - fractions[k]);
            for k in 0..n {
                let wrong_way = (rising && slopes[k] < 0.0) || (!rising && slopes[k] > 0.0);
                if !slopes[k].is_finite() || wrong_way {
                    slopes[k] = if k == 0 { secant(0) } else if k == n - 1 { secant(n - 2) } else { 0.5 * (secant(k - 1) + secant(k)) };
                }
            }
            for k in 0..n - 1 {
                let sec = secant(k);
                if sec == 0.0 {
                    slopes[k] = 0.0;
                    slopes[k + 1] = 0.0;
                    continue;
                }
                let (alpha, beta) = (slopes[k] / sec, slopes[k + 1] / sec);
                let norm = alpha * alpha + beta * beta;
                if norm > 9.0 {
                    let tau = 3.0 / norm.sqrt();
                    slopes[k] = tau * alpha * sec;
                    slopes[k + 1] = tau * beta * sec;
                }
            }
            let candidate = hermite_pcurve_at(&fractions, &values, &slopes, fixed_is_v, fixed).ok_or("hermite")?;
            let along: Vec<f64> = candidate.control_points.iter().map(|c| if fixed_is_v { c.x } else { c.y }).collect();
            let inside = along.iter().all(|value| *value >= s_lo && *value <= s_hi);
            let ordered = along.windows(2).all(|w| if rising { w[1] >= w[0] } else { w[1] <= w[0] });
            if inside && ordered {
                let gap = matched_gap_cells(surface, &candidate, edge, t0, t1, forward, &fractions, 0.5 * target).ok_or("gap read")?;
                last = gap;
                if gap <= target {
                    return Ok(candidate);
                }
            }
            stations *= 2;
        }
        Err(format!("row-stationed: target {target:.3e} not reached at 1024 stations: {last:.3e}"))
    };
    row_stationed().map_err(|row| format!("{by_edge}; {row}"))
}

/// Newton on `(C(s) - p) . C'(s) = 0` from `guess`, kept inside `[lo, hi]`.
fn project_on_row(row: &NurbsCurve, point: Vec3, guess: f64, lo: f64, hi: f64) -> Option<f64> {
    let mut s = guess.clamp(lo, hi);
    for _ in 0..50 {
        let d = row.derivatives(s, 2).ok()?;
        let (c, c1, c2) = (*d.first()?, *d.get(1)?, *d.get(2)?);
        let offset = c.sub(point);
        let g = offset.dot(c1);
        let dg = c1.dot(c1) + offset.dot(c2);
        if !(dg.abs() > 1e-300) {
            return None;
        }
        let next = (s - g / dg).clamp(lo, hi);
        if (next - s).abs() <= 1e-15 * (1.0 + s.abs()) {
            return Some(next);
        }
        s = next;
    }
    Some(s)
}

/// Piecewise-cubic Hermite in the fraction, as a degree-3 B-spline with
/// triple interior knots, on the row `fixed`.
fn hermite_pcurve(values: &[f64], slopes: &[f64], fixed_is_v: bool, fixed: f64) -> Option<NurbsCurve> {
    let segments = values.len() - 1;
    let h = 1.0 / segments as f64;
    let fractions: Vec<f64> = (0..=segments).map(|k| if k == segments { 1.0 } else { k as f64 * h }).collect();
    hermite_pcurve_at(&fractions, values, slopes, fixed_is_v, fixed)
}

/// [`hermite_pcurve`] through `values` at the given increasing `fractions`
/// (0 first, 1 last), with `slopes` in the fraction.
fn hermite_pcurve_at(fractions: &[f64], values: &[f64], slopes: &[f64], fixed_is_v: bool, fixed: f64) -> Option<NurbsCurve> {
    let segments = values.len() - 1;
    let point = |s: f64| if fixed_is_v { Vec4 { x: s, y: fixed, z: 0.0, w: 1.0 } } else { Vec4 { x: fixed, y: s, z: 0.0, w: 1.0 } };
    let mut controls = vec![point(values[0])];
    let mut knots = vec![0.0; 4];
    for k in 0..segments {
        let h = fractions[k + 1] - fractions[k];
        controls.push(point(values[k] + h * slopes[k] / 3.0));
        controls.push(point(values[k + 1] - h * slopes[k + 1] / 3.0));
        controls.push(point(values[k + 1]));
        if k + 1 < segments {
            knots.extend([fractions[k + 1]; 3]);
        }
    }
    knots.extend([1.0; 4]);
    NurbsCurve::new(3, knots, controls).ok()
}

/// [`matched_gap`] read in every cell between `fractions` (eight steps a
/// cell), so a narrow cell at a crowded end is read too; every cell whose
/// sampled reading reaches `refine_from` has its largest reading refined to
/// the local peak between its neighbouring steps (ternary search), since
/// sampled points alone can read a cell's peak low, and another cell's peak
/// may pass the one that sampled highest.
fn matched_gap_cells(surface: &NurbsSurface, pcurve: &NurbsCurve, edge: &NurbsCurve, t0: f64, t1: f64, forward: bool, fractions: &[f64], refine_from: f64) -> Option<f64> {
    let [q0, q1] = pcurve.domain().ok()?;
    let gap_at = |f: f64| -> Option<f64> {
        let uv = pcurve.evaluate(q0 + (q1 - q0) * f).ok()?;
        let on_surface = surface.evaluate_extended(uv.x, uv.y).ok()?;
        let t = if forward { t0 + (t1 - t0) * f } else { t1 - (t1 - t0) * f };
        let distance = on_surface.sub(edge.evaluate(t).ok()?).length();
        distance.is_finite().then_some(distance)
    };
    let mut worst = 0.0f64;
    for cell in fractions.windows(2) {
        let step = (cell[1] - cell[0]) / 8.0;
        let mut cell_worst = (0.0f64, cell[0]);
        for k in 0..=8 {
            let f = cell[0] + step * k as f64;
            let distance = gap_at(f)?;
            if distance > cell_worst.0 {
                cell_worst = (distance, f);
            }
        }
        let mut peak = cell_worst.0;
        if peak >= refine_from {
            let (mut low, mut high) = ((cell_worst.1 - step).max(cell[0]), (cell_worst.1 + step).min(cell[1]));
            for _ in 0..60 {
                let (m1, m2) = (low + (high - low) / 3.0, high - (high - low) / 3.0);
                let (g1, g2) = (gap_at(m1)?, gap_at(m2)?);
                peak = peak.max(g1).max(g2);
                if g1 >= g2 {
                    high = m2;
                } else {
                    low = m1;
                }
            }
        }
        worst = worst.max(peak);
    }
    Some(worst)
}

/// The largest same-parameter distance over `samples` + 1 fractions: the
/// surface at the pcurve's point against the edge at the same fraction.
fn matched_gap(surface: &NurbsSurface, pcurve: &NurbsCurve, edge: &NurbsCurve, t0: f64, t1: f64, forward: bool, samples: usize) -> Option<f64> {
    let [q0, q1] = pcurve.domain().ok()?;
    let mut worst = 0.0f64;
    for k in 0..=samples {
        let f = k as f64 / samples as f64;
        let uv = pcurve.evaluate(q0 + (q1 - q0) * f).ok()?;
        let on_surface = surface.evaluate_extended(uv.x, uv.y).ok()?;
        let t = if forward { t0 + (t1 - t0) * f } else { t1 - (t1 - t0) * f };
        let distance = on_surface.sub(edge.evaluate(t).ok()?).length();
        if !distance.is_finite() {
            return None;
        }
        worst = worst.max(distance);
    }
    Some(worst)
}

/// The trim's distance from its edge as POINT SETS: each lifted pcurve point
/// against the edge's curve on its own range `[t0, t1]`, at 65 fractions.
fn geometric_gap(surface: &NurbsSurface, pcurve: &NurbsCurve, edge: &NurbsCurve, t0: f64, t1: f64) -> Option<f64> {
    let [q0, q1] = pcurve.domain().ok()?;
    let mut worst = 0.0f64;
    for k in 0..=64 {
        let uv = pcurve.evaluate(q0 + (q1 - q0) * k as f64 / 64.0).ok()?;
        let point = surface.evaluate_extended(uv.x, uv.y).ok()?;
        worst = worst.max(distance_to_range(edge, t0, t1, point)?);
    }
    Some(worst)
}

/// `point`'s distance to `curve` on `[t0, t1]`: uniform stations plus the
/// knots and each knot span's quarter points, then a ternary refinement
/// around the nearest one.
fn distance_to_range(curve: &NurbsCurve, t0: f64, t1: f64, point: Vec3) -> Option<f64> {
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let at = |t: f64| curve.evaluate(t).ok().map(|q| q.sub(point).length());
    let mut candidates: Vec<f64> = (0..=128).map(|k| lo + (hi - lo) * k as f64 / 128.0).collect();
    let mut knots: Vec<f64> = curve.knots.iter().copied().filter(|k| *k > lo && *k < hi).collect();
    knots.push(lo);
    knots.push(hi);
    knots.sort_by(|a, b| a.total_cmp(b));
    knots.dedup();
    for pair in knots.windows(2) {
        for quarter in 0..4 {
            candidates.push(pair[0] + (pair[1] - pair[0]) * quarter as f64 / 4.0);
        }
    }
    candidates.sort_by(|a, b| a.total_cmp(b));
    candidates.dedup();
    let mut best = (0usize, f64::INFINITY);
    for (index, t) in candidates.iter().enumerate() {
        let d = at(*t)?;
        if d < best.1 {
            best = (index, d);
        }
    }
    let (mut a, mut b) = (candidates[best.0.saturating_sub(1)], candidates[(best.0 + 1).min(candidates.len() - 1)]);
    for _ in 0..60 {
        let (m1, m2) = (a + (b - a) / 3.0, b - (b - a) / 3.0);
        if at(m1)? <= at(m2)? {
            b = m2;
        } else {
            a = m1;
        }
    }
    Some(best.1.min(at(0.5 * (a + b))?))
}

fn vertex_diagonal(solid: &BrepSolid) -> f64 {
    let Some(first) = solid.vertices.first() else { return 0.0 };
    let (mut lo, mut hi) = (first.point, first.point);
    for vertex in &solid.vertices {
        let p = vertex.point;
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    hi.sub(lo).length()
}
