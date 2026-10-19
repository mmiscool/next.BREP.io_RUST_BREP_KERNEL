//! Blend-band body certificate (lane K2, `BREP_SOUNDNESS_BLEND_BODY`, on by
//! default, `=0` to disable): the analytic body certificate's proof over a
//! body whose faces are native planes and cylinders plus NURBS PATCHES, each
//! proved fold-free from its own control net and kept to one side of every
//! plane it meets. That is the shape a fillet or chamfer leaves on a
//! plane-and-cylinder body; nothing here reads who built it. Anything this
//! module cannot prove declines to the full scan.
use super::analytic_body::{carrier_of, Carrier, Plane};
use super::tube_grid::{bezier_spans, injective, split_u, Cell, MAX_CELLS, SPLIT_U};
use super::*;
use crate::Vec4;

pub(super) type Decline = &'static str;

/// A cell's homogeneous control net: rows k (u), columns l (v), `[wx, wy, wz, w]`.
pub(super) type Net = Vec<Vec<[f64; 4]>>;

pub(super) fn enabled() -> bool {
    // Same gate as the analytic body: with either trim switch off the scan's
    // notion of material is the whole carrier.
    if !super::trim_samples_enabled() || !super::node_membership_enabled() {
        return false;
    }
    // On by default (promoted 2026-10-16); `BREP_SOUNDNESS_BLEND_BODY=0`
    // (any value but 1/on) is the escape hatch back to the full scan.
    match std::env::var("BREP_SOUNDNESS_BLEND_BODY") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

/// Lane K2 closed-patch row (`BREP_SOUNDNESS_BLEND_CLOSED`, on by default,
/// `=0` to disable): a patch
/// closed along its rational direction (a full-turn chamfer cone) is glued
/// into a cyclic grid, as tube_grid glues a ring, instead of declining on its
/// own seam.
pub(super) fn closed_enabled() -> bool {
    // On by default (promoted 2026-10-16); `BREP_SOUNDNESS_BLEND_CLOSED=0`
    // (any value but 1/on) declines a closed patch on its seam again.
    match std::env::var("BREP_SOUNDNESS_BLEND_CLOSED") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    match verdict(solid, options) {
        Ok(_) => true,
        Err(reason) => {
            if std::env::var_os("BREP_DEBUG_BLEND_BODY").is_some() {
                eprintln!("soundness.blend_body decline: {reason}");
            }
            false
        }
    }
}

// What is proved, on top of the analytic body's notes (analytic_body.rs).
//
// - A patch is a NURBS face rational in one direction only (weights constant
//   along the other) and already Bezier in that direction. Its trims' control
//   hull in the chart, clamped to the domain, is a box holding the trimmed
//   face; the patch is restricted to that box by de Casteljau in homogeneous
//   coordinates, so the restricted cells hold the trimmed face (positive
//   weights: convex hulls). Every 2x2 block of cells is injective by its
//   tangent cones and every other cell pair is separated by more than `tau`
//   (tube_grid's proof, without the cyclic glue): the patch does not fold.
// - A patch and a face it shares no edge with: world boxes (the restricted
//   cells' control points) separated by more than `tau` on a world axis.
// - A patch and a plane it shares an edge with: every restricted control point
//   lies on ONE side of the plane, within `tau`. This is weaker than
//   tube_grid's cap rule on purpose: a blend tangent to a plane along its rail
//   has interior control points ON the plane (a conic's corner point), so they
//   may sit within `tau` of it on either side. It covers the scan's verdict by
//   the same argument as the cap rule: a confirmed crossing needs a straddle
//   of the band (1000 `tau`) one way or the other, and no point of the patch
//   is more than `tau` across the plane.
// - Every other adjacent pair with a patch (patch-cylinder, patch-patch)
//   declines, unless `blend_curved.rs` is on: it adds patches rational in
//   both directions, and a patch against a cylinder or a torus it meets.
//   Pairs of native faces are the analytic body's proof, unchanged.

/// A patch face: its restricted cells and their control points.
pub(super) struct Patch {
    pub(super) points: Vec<Vec3>,
    /// The restricted cells' homogeneous nets (`blend_curved.rs` reads them).
    pub(super) nets: Vec<Net>,
    pub(super) lo: Vec3,
    pub(super) hi: Vec3,
}

/// The patch net oriented rational-in-u: `(net, knots_u, knots_v, pu, pv,
/// swapped)`, or why it is not a patch this proof reads.
fn oriented(s: &NurbsSurface) -> Result<(Vec<Vec<Vec4>>, Vec<f64>, Vec<f64>, usize, usize, bool), Decline> {
    let net = &s.control_points;
    if s.degree_u == 0 || s.degree_v == 0 || net.len() < 2 || net[0].len() < 2 || net.iter().any(|row| row.len() != net[0].len()) {
        return Err("a patch net is not a grid");
    }
    if net.iter().flatten().any(|c| !(c.w > 0.) || !c.w.is_finite() || !c.x.is_finite() || !c.y.is_finite() || !c.z.is_finite()) {
        return Err("a patch control point is not finite with a positive weight");
    }
    if net.iter().all(|row| row.iter().all(|c| c.w == row[0].w)) {
        return Ok((net.clone(), s.knots_u.clone(), s.knots_v.clone(), s.degree_u, s.degree_v, false));
    }
    if (0..net[0].len()).all(|l| net.iter().all(|row| row[l].w == net[0][l].w)) {
        let transposed = (0..net[0].len()).map(|l| net.iter().map(|row| row[l]).collect()).collect();
        return Ok((transposed, s.knots_v.clone(), s.knots_u.clone(), s.degree_v, s.degree_u, true));
    }
    Err(BOTH_WAYS)
}

const BOTH_WAYS: Decline = "a patch is rational in both directions";

/// Seam agreement for a closed patch, relative to its size (the carriers'
/// recognition slack).
const CLOSED_DELTA: f64 = 1e-12;

/// Cheap shape checks, no cells: a face this proof could read as a patch.
pub(super) fn screen(record: &FaceRecord) -> Result<(), Decline> {
    let (net, ku, kv, pu, pv, _) = match oriented(&record.surface) {
        Err(BOTH_WAYS) if super::blend_curved::enabled() => return super::blend_curved::screen(record),
        oriented => oriented?,
    };
    if ku.len() != net.len() + pu + 1 || kv.len() != net[0].len() + pv + 1 {
        return Err("a patch's knots do not fit its net");
    }
    let (a, b) = (ku[0], ku[ku.len() - 1]);
    if !(b > a) || ku[..=pu].iter().any(|k| *k != a) || ku[ku.len() - pu - 1..].iter().any(|k| *k != b) {
        return Err("a patch's rational knots are not clamped");
    }
    let mut interior = ku[pu + 1..ku.len() - pu - 1].to_vec();
    interior.dedup();
    if interior.iter().any(|k| ku.iter().filter(|x| *x == k).count() != pu) {
        return Err("a patch is not Bezier in its rational direction");
    }
    if record.loops.is_empty() || record.loops.iter().flat_map(|l| &l.coedges).next().is_none() {
        return Err("a patch has no trim");
    }
    Ok(())
}

/// de Casteljau at `t`: the left and right control polygons.
fn split(points: &[[f64; 4]], t: f64) -> (Vec<[f64; 4]>, Vec<[f64; 4]>) {
    let n = points.len();
    let mut level = points.to_vec();
    let mut left = Vec::with_capacity(n);
    let mut right = Vec::with_capacity(n);
    left.push(level[0]);
    right.push(level[n - 1]);
    for _ in 1..n {
        level = (0..level.len() - 1).map(|i| std::array::from_fn(|c| (1. - t) * level[i][c] + t * level[i + 1][c])).collect();
        left.push(level[0]);
        right.push(level[level.len() - 1]);
    }
    right.reverse();
    (left, right)
}

/// The control polygon of the piece over local parameters `[t0, t1]`.
pub(super) fn segment(points: &[[f64; 4]], t0: f64, t1: f64) -> Vec<[f64; 4]> {
    let mut piece = points.to_vec();
    if t0 > 0. {
        piece = split(&piece, t0).1;
    }
    let t = if t0 > 0. { (t1 - t0) / (1. - t0) } else { t1 };
    if t < 1. {
        piece = split(&piece, t).0;
    }
    piece
}

/// The distinct breakpoints of a clamped knot vector.
pub(super) fn breaks(knots: &[f64]) -> Vec<f64> {
    let mut out = knots.to_vec();
    out.dedup();
    out
}

/// The overlap of span `[a, b]` with `[lo, hi]` as local parameters.
pub(super) fn local(a: f64, b: f64, lo: f64, hi: f64) -> Option<(f64, f64)> {
    let (from, to) = (a.max(lo), b.min(hi));
    (to > from).then(|| (((from - a) / (b - a)).clamp(0., 1.), ((to - a) / (b - a)).clamp(0., 1.)))
}

/// The face restricted to its trims' chart box, as cells, proved fold-free.
pub(super) fn patch(record: &FaceRecord, tau: f64) -> Result<Patch, Decline> {
    if super::blend_curved::enabled() && matches!(oriented(&record.surface), Err(BOTH_WAYS)) {
        let nets = super::blend_curved::patch(record, tau)?;
        let points: Vec<Vec3> = nets.iter().flatten().flatten().map(|h| Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])).collect();
        let lo = points.iter().fold(points[0], |m, q| Vec3::new(m.x.min(q.x), m.y.min(q.y), m.z.min(q.z)));
        let hi = points.iter().fold(points[0], |m, q| Vec3::new(m.x.max(q.x), m.y.max(q.y), m.z.max(q.z)));
        return Ok(Patch { points, nets, lo, hi });
    }
    screen(record)?;
    let (net, ku, kv, pu, pv, swapped) = oriented(&record.surface)?;
    // The trims' control hull in the chart (positive weights: it holds every
    // trim, so the trimmed face), clamped to the domain.
    let mut bounds = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
    for c in record.loops.iter().flat_map(|l| &l.coedges).flat_map(|ce| &ce.pcurve.control_points) {
        if !(c.w > 0.) {
            return Err("a patch trim weight is not positive");
        }
        let (mut u, mut v) = (c.x / c.w, c.y / c.w);
        if swapped {
            (u, v) = (v, u);
        }
        if !u.is_finite() || !v.is_finite() {
            return Err("a patch trim is not finite");
        }
        bounds = [bounds[0].min(u), bounds[1].max(u), bounds[2].min(v), bounds[3].max(v)];
    }
    let (bu, bv) = (breaks(&ku), breaks(&kv));
    let [u0, u1, v0, v1] = [
        bounds[0].max(bu[0]),
        bounds[1].min(bu[bu.len() - 1]),
        bounds[2].max(bv[0]),
        bounds[3].min(bv[bv.len() - 1]),
    ];
    if !(u1 > u0 && v1 > v0) {
        return Err("a patch trim box is empty");
    }
    // v (polynomial) to Bezier spans per u row; u is already Bezier.
    let rows: Vec<Vec<Vec<Vec3>>> = net
        .iter()
        .map(|row| {
            let euclid: Vec<Vec3> = row.iter().map(|c| Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w)).collect();
            bezier_spans(&kv, &euclid, pv)
        })
        .collect::<Option<_>>()
        .ok_or("a patch's polynomial knots are not a clamped spline")?;
    if bv.len() - 1 != rows[0].len() || bu.len() - 1 != (net.len() - 1) / pu {
        return Err("a patch's spans do not match its knots");
    }
    let mut grid: Vec<Vec<Cell>> = Vec::new();
    let mut nets: Vec<Net> = Vec::new();
    for span in 0..bu.len() - 1 {
        let Some((s0, s1)) = local(bu[span], bu[span + 1], u0, u1) else { continue };
        let mut pieces: Vec<Vec<Cell>> = (0..SPLIT_U).map(|_| Vec::new()).collect();
        for j in 0..bv.len() - 1 {
            let Some((r0, r1)) = local(bv[j], bv[j + 1], v0, v1) else { continue };
            // Restrict v on each row in Euclidean form (polynomial in v), then
            // weight it: the row's weight stays exact in every column.
            let cell: Vec<Vec<[f64; 4]>> = (0..=pu)
                .map(|k| {
                    let w = net[span * pu + k][0].w;
                    let row: Vec<[f64; 4]> = rows[span * pu + k][j].iter().map(|p| [p.x, p.y, p.z, 1.]).collect();
                    segment(&row, r0, r1).into_iter().map(|q| [q[0] * w, q[1] * w, q[2] * w, w]).collect()
                })
                .collect();
            // Restrict u on each column (homogeneous; the same arithmetic on the
            // same weights in every column).
            let columns: Vec<Vec<[f64; 4]>> = (0..=pv).map(|l| segment(&cell.iter().map(|row| row[l]).collect::<Vec<_>>(), s0, s1)).collect();
            let cell: Vec<Vec<[f64; 4]>> = (0..=pu).map(|k| columns.iter().map(|column| column[k]).collect()).collect();
            for (q, (piece, weights)) in split_u(&cell, SPLIT_U).ok_or("a restricted patch cell lost its weights")?.into_iter().enumerate() {
                let net: Net = piece.iter().zip(&weights).map(|(row, w)| row.iter().map(|p| [p.x * w, p.y * w, p.z * w, *w]).collect()).collect();
                let net = if swapped { net_transposed(&net) } else { net };
                nets.push(net);
                pieces[q].push(Cell::new(piece, weights).ok_or("a degenerate patch cell")?);
            }
        }
        grid.extend(pieces);
    }
    let (nu, nv) = (grid.len(), grid.first().map_or(0, Vec::len));
    if nu == 0 || nv == 0 || nu * nv > MAX_CELLS || grid.iter().any(|column| column.len() != nv) {
        return Err("a patch cell grid outside the proof's size");
    }
    // Closed along u: the box spans the whole u range and the net's first and
    // last rows agree to `CLOSED_DELTA` of its size (the seam is one curve).
    let closed = closed_enabled() && u0 == bu[0] && u1 == bu[bu.len() - 1] && {
        let size = net.iter().flatten().fold(0f64, |m, c| m.max((c.x / c.w).abs()).max((c.y / c.w).abs()).max((c.z / c.w).abs()));
        net[0].iter().zip(&net[net.len() - 1]).all(|(a, b)| {
            [a.x / a.w - b.x / b.w, a.y / a.w - b.y / b.w, a.z / a.w - b.z / b.w].iter().all(|d| d.abs() <= CLOSED_DELTA * size)
                && (a.w - b.w).abs() <= CLOSED_DELTA * a.w.abs().max(b.w.abs())
        })
    };
    injective(&grid, closed, tau)?;
    let points: Vec<Vec3> = grid.iter().flatten().flat_map(|cell| cell.net.iter().flatten().copied()).collect();
    let lo = points.iter().fold(points[0], |m, q| Vec3::new(m.x.min(q.x), m.y.min(q.y), m.z.min(q.z)));
    let hi = points.iter().fold(points[0], |m, q| Vec3::new(m.x.max(q.x), m.y.max(q.y), m.z.max(q.z)));
    Ok(Patch { points, nets, lo, hi })
}

fn net_transposed(net: &Net) -> Net {
    (0..net[0].len()).map(|l| net.iter().map(|row| row[l]).collect()).collect()
}

/// Every point lies on one side of the plane, within `tau`.
pub(super) fn one_sided(points: &[Vec3], plane: &Plane, tau: f64) -> bool {
    let _timing = crate::blend::BlendTimer::start("timing.soundness.one_sided_plane_ms");
    let (normal, level) = (plane.normal(), plane.normal().dot(plane.at_origin()));
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in points {
        let g = normal.dot(*p) - level;
        lo = lo.min(g);
        hi = hi.max(g);
    }
    lo >= -tau || hi <= tau
}

/// `Ok(pairs proved)` when the body is proved free of crossings and folds.
pub(super) fn verdict(solid: &BrepSolid, options: SelfIntersectionOptions) -> Result<usize, Decline> {
    let _timing = crate::blend::BlendTimer::start("timing.soundness.blend_patch_verdict_ms");
    // Cheap gates first, no cells: at least one patch, every other face a
    // native carrier, every patch shaped for this proof, and no patch next to
    // a cylinder or another patch.
    let records: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    // The analytic body's own bound, before any carrier is read.
    if records.is_empty() || records.len() > 256 {
        return Err("face count outside 1..=256");
    }
    let mut is_patch = HashMap::default();
    let mut cylinder = HashMap::default();
    for record in &records {
        match carrier_of(&record.surface) {
            Some(Carrier::Cylinder(_)) => {
                cylinder.insert(record.id, ());
            }
            Some(Carrier::Plane(_)) => {}
            None => {
                screen(record)?;
                is_patch.insert(record.id, ());
            }
        }
    }
    if is_patch.is_empty() {
        return Err("no patch face (the analytic body's case)");
    }
    let mut users: HashMap<u64, Vec<u64>> = HashMap::default();
    for record in &records {
        for coedge in record.loops.iter().flat_map(|l| &l.coedges) {
            users.entry(coedge.edge_id).or_default().push(record.id);
        }
    }
    for faces in users.values() {
        if let [a, b] = faces[..] {
            if a != b && (is_patch.contains_key(&a) || is_patch.contains_key(&b)) {
                let other = if is_patch.contains_key(&a) { b } else { a };
                if is_patch.contains_key(&other) || cylinder.contains_key(&other) {
                    // A curved neighbour is analytic_body's call, face by face.
                    if !super::blend_curved::enabled() {
                        return Err("a patch meets a cylinder or another patch");
                    }
                }
            }
        }
    }
    super::analytic_body::verdict_with_patches(solid, options)
}

