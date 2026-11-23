//! A curve proved clear of a surface: no point of the curve within a tolerance of any point of the
//! surface, shown by Bézier hull boxes.
//!
//! A general curve-surface search (`intersect_curve_surface`) only ever returns a hit whose gap is
//! within its tolerance. Each Bézier piece of the curve lies in the hull of its control points, and
//! each Bézier patch of the surface in the hull of its own (weights positive); a piece and a patch
//! whose hull boxes stand apart by more than the tolerance (and a roundoff cushion) on some axis hold
//! no such pair, and a pair that does not stand apart is halved (the larger one, by de Casteljau)
//! until it does. Where every pair is cleared, the search's answer is known to be empty; where any
//! pair will not clear within the budget, the proof says nothing. Callers (the imprint's pierce
//! search, the point classifier's rays) each put it behind their own switch, and
//! `BREP_CURVE_CLEAR_CHECK=1` makes both run the search anyway and stop on any disagreement.
use crate::{NurbsCurve, NurbsSurface, Vec4};

/// `BREP_CURVE_CLEAR_CHECK=1` (default off): callers run the full search as well and stop on a
/// curve the proof clears that the search finds a hit on.
pub fn check_mode() -> bool {
    switch!("BREP_CURVE_CLEAR_CHECK", false)
}

/// The check mode's verdict: the proof cleared the curve and the search still found `hits`.
pub fn disagreement(lane: &str, hits: usize) -> ! {
    panic!("curve clear check ({lane}): the proof cleared a curve the full search hits {hits} times")
}

/// Close pairs met plus pairs halved before the proof gives up (each halving costs two hull boxes).
const SPLIT_BUDGET: usize = 4096;
/// Halvings along one branch before the proof gives up (a pair this deep is near or touching).
const DEPTH_LIMIT: usize = 18;

type Box3 = ([f64; 3], [f64; 3]);

fn hull_box<'a>(points: impl IntoIterator<Item = &'a Vec4>) -> Option<Box3> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
        if !(p.w > 0.0) || !p.w.is_finite() {
            return None; // the hull holds the curve only for positive weights
        }
        let q = [p.x / p.w, p.y / p.w, p.z / p.w];
        for k in 0..3 {
            if !q[k].is_finite() {
                return None;
            }
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    Some((lo, hi))
}

fn apart(a: &Box3, b: &Box3, margin: f64) -> bool {
    (0..3).any(|k| a.0[k] > b.1[k] + margin || b.0[k] > a.1[k] + margin)
}

fn extent(b: &Box3) -> f64 {
    (0..3).map(|k| b.1[k] - b.0[k]).fold(0.0, f64::max)
}

fn half(a: Vec4, b: Vec4) -> Vec4 {
    a.add(b).scale(0.5)
}

/// De Casteljau at the middle, in homogeneous coordinates: the two halves' control points.
fn halve(points: &[Vec4]) -> (Vec<Vec4>, Vec<Vec4>) {
    let mut work = points.to_vec();
    let n = work.len();
    let mut left = Vec::with_capacity(n);
    let mut right = vec![work[n - 1]; n];
    left.push(work[0]);
    right[n - 1] = work[n - 1];
    for level in 1..n {
        for i in 0..n - level {
            work[i] = half(work[i], work[i + 1]);
        }
        left.push(work[0]);
        right[n - 1 - level] = work[n - 1 - level];
    }
    (left, right)
}

/// A Bézier patch as rows along u, each row a column of points along v.
type Patch = Vec<Vec<Vec4>>;

fn halve_u(patch: &Patch) -> (Patch, Patch) {
    let columns = patch[0].len();
    let (mut a, mut b) = (vec![Vec::with_capacity(columns); patch.len()], vec![Vec::with_capacity(columns); patch.len()]);
    for j in 0..columns {
        let column: Vec<Vec4> = patch.iter().map(|row| row[j]).collect();
        let (l, r) = halve(&column);
        for i in 0..patch.len() {
            a[i].push(l[i]);
            b[i].push(r[i]);
        }
    }
    (a, b)
}

fn halve_v(patch: &Patch) -> (Patch, Patch) {
    let mut a = Vec::with_capacity(patch.len());
    let mut b = Vec::with_capacity(patch.len());
    for row in patch {
        let (l, r) = halve(row);
        a.push(l);
        b.push(r);
    }
    (a, b)
}

/// The longer way across the patch's net: along u (rows) or along v.
fn longer_along_u(patch: &Patch) -> bool {
    let dist = |p: Vec4, q: Vec4| {
        let (p, q) = ([p.x / p.w, p.y / p.w, p.z / p.w], [q.x / q.w, q.y / q.w, q.z / q.w]);
        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
    };
    let (rows, columns) = (patch.len(), patch[0].len());
    let along_u = (0..columns).map(|j| dist(patch[0][j], patch[rows - 1][j])).fold(0.0, f64::max);
    let along_v = (0..rows).map(|i| dist(patch[i][0], patch[i][columns - 1])).fold(0.0, f64::max);
    along_u >= along_v
}

/// A surface's Bézier patches (each column decomposed along u, then each row of a u span along
/// v) and their hull boxes: built once per surface ([`NurbsSurface::bezier_patches`]). None where
/// the net does not decompose or a patch weight is not positive.
pub struct SurfacePatches {
    patches: Vec<Patch>,
    boxes: Vec<Box3>,
    /// Patches along v per u span (the patches run u span by u span).
    v_spans: usize,
}

impl std::fmt::Debug for SurfacePatches {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SurfacePatches({})", self.patches.len())
    }
}

fn surface_patches(surface: &NurbsSurface) -> Option<SurfacePatches> {
    let zero = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
    let blend = |a: Vec4, b: Vec4, alpha: f64| a.scale(alpha).add(b.scale(1.0 - alpha));
    let columns = surface.control_points.first()?.len();
    if columns == 0 || surface.control_points.iter().any(|row| row.len() != columns) {
        return None;
    }
    let mut fibres = Vec::with_capacity(columns);
    for column in 0..columns {
        let controls: Vec<Vec4> = surface.control_points.iter().map(|row| row[column]).collect();
        fibres.push(crate::curve::bezier_decompose_raw(&surface.knots_u, &controls, surface.degree_u, zero, blend)?);
    }
    let mut patches: Vec<Patch> = Vec::new();
    for u_span in 0..fibres[0].len() {
        let mut across = Vec::with_capacity(surface.degree_u + 1);
        for row in 0..=surface.degree_u {
            let controls: Vec<Vec4> = fibres.iter().map(|fibre| fibre[u_span][row]).collect();
            across.push(crate::curve::bezier_decompose_raw(&surface.knots_v, &controls, surface.degree_v, zero, blend)?);
        }
        for v_span in 0..across[0].len() {
            patches.push(across.iter().map(|row| row[v_span].clone()).collect());
        }
    }
    let boxes = patches.iter().map(|patch| hull_box(patch.iter().flatten())).collect::<Option<Vec<_>>>()?;
    // `fibres[0]` holds one polygon per u span.
    let v_spans = patches.len() / fibres[0].len().max(1);
    Some(SurfacePatches { patches, boxes, v_spans })
}

/// Is every point of `curve` farther than `tolerance` from every point of `surface`, by the
/// hull proof above? False where it cannot be shown.
pub fn curve_clear_of_surface(curve: &NurbsCurve, surface: &NurbsSurface, tolerance: f64) -> bool {
    curve_clear_within(curve, surface, tolerance, SPLIT_BUDGET)
}

/// [`curve_clear_of_surface`] with its budget given.
pub fn curve_clear_within(curve: &NurbsCurve, surface: &NurbsSurface, tolerance: f64, split_budget: usize) -> bool {
    if !(tolerance.is_finite() && tolerance >= 0.0) || surface.control_points.is_empty() || surface.control_points[0].is_empty() {
        return false;
    }
    let zero = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
    let blend = |a: Vec4, b: Vec4, alpha: f64| a.scale(alpha).add(b.scale(1.0 - alpha));
    let Some(pieces) = crate::curve::bezier_decompose_raw(&curve.knots, &curve.control_points, curve.degree, zero, blend) else { return false };
    // The carrier's patches and their boxes, read once per surface.
    let Some(carrier) = surface.bezier_patches.get_or_init(|| surface_patches(surface).map(std::sync::Arc::new)).clone() else { return false };
    let (patches, patch_boxes) = (&carrier.patches, &carrier.boxes);
    // Boxes, and the scale for the roundoff cushion.
    let mut piece_boxes = Vec::with_capacity(pieces.len());
    for piece in &pieces {
        let Some(b) = hull_box(piece) else { return false };
        piece_boxes.push(b);
    }
    let scale = piece_boxes
        .iter()
        .chain(patch_boxes.iter())
        .flat_map(|b| b.0.iter().chain(&b.1))
        .fold(1.0f64, |m, x| m.max(x.abs()));
    let degrees = (curve.degree + 1 + surface.degree_u + 1 + surface.degree_v + 1) as f64;
    let margin = tolerance + 1024.0 * f64::EPSILON * scale * degrees * (DEPTH_LIMIT as f64 + 1.0);
    // Each overlapping pair is settled on its own small stack, one at a time, against one budget
    // that also counts the pairs themselves: memory stays at one branch, and a net with too many
    // close pairs gives up early rather than queue them all.
    let mut budget = split_budget;
    let mut stack: Vec<(Vec<Vec4>, Patch, usize)> = Vec::new();
    for (i, cb) in piece_boxes.iter().enumerate() {
        for (j, sb) in patch_boxes.iter().enumerate() {
            if apart(cb, sb, margin) {
                continue;
            }
            if budget == 0 {
                return false;
            }
            budget -= 1;
            stack.clear();
            stack.push((pieces[i].clone(), patches[j].clone(), 0));
            while let Some((piece, patch, depth)) = stack.pop() {
                let (Some(cb), Some(sb)) = (hull_box(&piece), hull_box(patch.iter().flatten())) else { return false };
                if apart(&cb, &sb, margin) {
                    continue;
                }
                if depth >= DEPTH_LIMIT || budget == 0 {
                    return false;
                }
                budget -= 1;
                if extent(&cb) >= extent(&sb) {
                    let (l, r) = halve(&piece);
                    stack.push((l, patch.clone(), depth + 1));
                    stack.push((r, patch, depth + 1));
                } else {
                    let (a, b) = if longer_along_u(&patch) { halve_u(&patch) } else { halve_v(&patch) };
                    stack.push((piece.clone(), a, depth + 1));
                    stack.push((piece, b, depth + 1));
                }
            }
        }
    }
    true
}

/// The control polygon of `polygon` (a Bézier curve over [0, 1]) restricted
/// to `[low, high]`, by two de Casteljau splits.
fn bezier_piece(polygon: &[Vec4], low: f64, high: f64) -> Vec<Vec4> {
    let split = |points: &[Vec4], t: f64| -> (Vec<Vec4>, Vec<Vec4>) {
        let mut work = points.to_vec();
        let n = work.len();
        let mut left = Vec::with_capacity(n);
        let mut right = vec![work[n - 1]; n];
        left.push(work[0]);
        for level in 1..n {
            for k in 0..n - level {
                work[k] = work[k].scale(1.0 - t).add(work[k + 1].scale(t));
            }
            left.push(work[0]);
            right[n - 1 - level] = work[n - 1 - level];
        }
        (left, right)
    };
    let head = if high < 1.0 { split(polygon, high).0 } else { polygon.to_vec() };
    if low > 0.0 && high > 0.0 { split(&head, low / high).1 } else { head }
}

/// The hull box of the piece `[u0, u1] × [v0, v1]` (fractions of the span,
/// each in [0, 1]) of the Bézier patch of `surface`'s knot spans `u_span`,
/// `v_span` ([`NurbsSurface::bezier_patches`], built once per surface), as
/// (low, high); `None` where the net does not decompose, a span is out of
/// range or a weight is not positive.
pub fn patch_piece_box(surface: &NurbsSurface, u_span: usize, v_span: usize, range_u: [f64; 2], range_v: [f64; 2]) -> Option<([f64; 3], [f64; 3])> {
    let (controls, _) = patch_piece_controls(surface, u_span, v_span, range_u, range_v)?;
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for point in controls {
        for (k, value) in [point.x, point.y, point.z].into_iter().enumerate() {
            lo[k] = lo[k].min(value);
            hi[k] = hi[k].max(value);
        }
    }
    Some((lo, hi))
}

/// How many Bezier patches `surface` splits into along u and along v (its
/// knot spans), `None` when the patches cannot be read.
pub fn patch_span_counts(surface: &NurbsSurface) -> Option<(usize, usize)> {
    let carrier = surface.bezier_patches.get_or_init(|| surface_patches(surface).map(std::sync::Arc::new)).clone()?;
    (carrier.v_spans > 0).then(|| (carrier.patches.len() / carrier.v_spans, carrier.v_spans))
}

/// The control points (Cartesian, row-major along u, `columns` per row) of
/// the piece `[u0, u1] × [v0, v1]` of the Bézier patch of `surface`'s knot
/// spans `u_span`, `v_span`, and `columns`; `None` as [`patch_piece_box`].
pub fn patch_piece_controls(surface: &NurbsSurface, u_span: usize, v_span: usize, [u0, u1]: [f64; 2], [v0, v1]: [f64; 2]) -> Option<(Vec<crate::Vec3>, usize)> {
    let carrier = surface.bezier_patches.get_or_init(|| surface_patches(surface).map(std::sync::Arc::new)).clone()?;
    if v_span >= carrier.v_spans {
        return None;
    }
    let patch = carrier.patches.get(u_span * carrier.v_spans + v_span)?;
    // patch[a][b]: a along u, b along v.
    let (rows, columns) = (patch.len(), patch.first()?.len());
    let mut across_u: Vec<Vec<Vec4>> = vec![Vec::with_capacity(rows); columns];
    for (b, fibre) in across_u.iter_mut().enumerate() {
        let polygon: Vec<Vec4> = (0..rows).map(|a| patch[a][b]).collect();
        *fibre = bezier_piece(&polygon, u0, u1);
    }
    let mut controls = Vec::with_capacity(rows * columns);
    for a in 0..rows {
        let polygon: Vec<Vec4> = (0..columns).map(|b| across_u[b][a]).collect();
        for control in bezier_piece(&polygon, v0, v1) {
            if !(control.w > 0.0) || !control.w.is_finite() {
                return None;
            }
            let point = crate::Vec3::new(control.x / control.w, control.y / control.w, control.z / control.w);
            if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite()) {
                return None;
            }
            controls.push(point);
        }
    }
    Some((controls, columns))
}

