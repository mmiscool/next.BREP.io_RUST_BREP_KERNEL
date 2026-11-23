//! Disjoint balls certificate (`BREP_SOUNDNESS_DISJOINT_BALLS`, default on since
//! the 2026-11-21 promotion, `=0` restores the full scan;
//! `BREP_SOUNDNESS_DISJOINT_BALLS_CHECK=1` runs the full scan as well, returns
//! ITS report, and says so on stderr when the scan flags a body the
//! certificate passed).
//!
//! A body, of any number of shells, whose every face is a native ball (the
//! net `make_sphere_surface` builds, carried rigidly, as the concentric balls
//! certificate reads it) with its trims inside the chart's domain, and whose
//! balls are pairwise the SAME sphere or share NO point: their centres apart by
//! more than the sum of their radii, or one inside the other, the distance of
//! the centres under the difference of the radii, each by more than the scan's
//! straddle band. Then:
//! - two faces on spheres that share no point have no point in common, so no
//!   crossing;
//! - two faces on one sphere lie on one carrier, which never straddles itself
//!   (the scan confirms a crossing only where each carrier passes through the
//!   other by more than the band; the recognition slack is checked to be far
//!   under it);
//! - a native ball's chart is injective away from the seam and poles the full
//!   scan identifies too, and every trim control lies in its domain, so no
//!   face folds.
//! So the body holds no crossing and no fold for the full scan to confirm,
//! and its empty report is returned without meshing. Nested or scattered whole
//! balls (brep-bench boolean/nested-shells: five balls, each its own shell)
//! were meshed whole at every acceptance. Anything else declines.
use super::*;

macro_rules! decline {
    () => {{
        if std::env::var_os("BREP_DEBUG_DISJOINT_BALLS").is_some() {
            eprintln!("soundness.disjoint_balls decline line={}", line!());
        }
        return false;
    }};
}

/// Recognition slack relative to the body's size.
const DELTA: f64 = 1e-12;

pub(super) fn enabled() -> bool {
    crate::switch!("BREP_SOUNDNESS_DISJOINT_BALLS", true)
}

pub(super) fn check_mode() -> bool {
    crate::switch!("BREP_SOUNDNESS_DISJOINT_BALLS_CHECK", false)
}


pub fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    // Any positive bands: the proof reads the scan's own straddle band (a
    // scoped acceptance hands a subset the whole result's bands).
    let readings = [options.chord_tolerance, options.straddle_band, options.min_crossing_length, options.weld];
    if !readings.iter().all(|value| value.is_finite() && *value > 0.0) {
        decline!();
    }
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    if faces.is_empty() {
        decline!();
    }
    // Each face a native ball (its trims inside the chart's domain) or a
    // native plane (an affine chart, injective everywhere).
    let mut kinds: Vec<Kind> = Vec::with_capacity(faces.len());
    for face in &faces {
        if let Some((centre, radius)) = sphere_certificate::native_ball_geometry(&face.surface)
            .or_else(|| concentric_balls::rebuilt_ball_with(&face.surface, true))
        {
            if !(radius > 0.0) || !radius.is_finite() || ![centre.x, centre.y, centre.z].iter().all(|v| v.is_finite()) {
                decline!();
            }
            if !concentric_balls::trims_inside_domain(face) {
                decline!();
            }
            kinds.push(Kind::Ball(centre, radius));
            continue;
        }
        match super::analytic_body::carrier_of(&face.surface) {
            Some(super::analytic_body::Carrier::Plane(plane)) => {
                let (normal, origin) = (plane.normal(), plane.at_origin());
                if !(normal.length() > 0.5) || ![origin.x, origin.y, origin.z].iter().all(|v| v.is_finite()) {
                    decline!();
                }
                kinds.push(Kind::Plane(plane));
            }
            _ => decline!(),
        }
    }
    if !kinds.iter().any(|kind| matches!(kind, Kind::Ball(..))) {
        decline!(); // planes alone are the analytic body certificate's
    }
    // The body's size from every carrier control: the recognitions' slack
    // (a plane's twist, a ball's net) is relative to it.
    let mut size = kinds
        .iter()
        .map(|kind| match kind {
            Kind::Ball(centre, radius) => centre.length() + radius,
            Kind::Plane(plane) => plane.at_origin().length(),
        })
        .fold(1.0, f64::max);
    for face in &faces {
        for control in face.surface.control_points.iter().flatten() {
            let Ok(point) = control.point() else { decline!() };
            size = size.max(point.length());
        }
    }
    let slack = DELTA * size;
    let band = options.straddle_band;
    // The recognition slack must be far under the band the scan confirms by.
    if !(slack * 1e3 < band) {
        decline!();
    }
    let depth = band / 8.0;
    let mut boundaries: Vec<Option<Option<(Vec<Cell>, Vec<Span>)>>> = (0..faces.len()).map(|_| None).collect();
    // A ball face's cells and pcurve spans, built once, at its first such pair.
    let boundary = |k: usize, boundaries: &mut Vec<Option<Option<(Vec<Cell>, Vec<Span>)>>>| {
        if boundaries[k].is_none() {
            let built = cells(&faces[k].surface).and_then(|cells| {
                let mut spans = Vec::new();
                for coedge in faces[k].loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                    spans.extend(pcurve_spans(&coedge.pcurve)?);
                }
                Some((cells, spans))
            });
            boundaries[k] = Some(built);
        }
    };
    // Whether ball face `k` keeps to one side of the plane `(x − c_k)·e = level`.
    let ball_side = |k: usize, e: Vec3, level: f64, delta: f64, boundaries: &mut Vec<Option<Option<(Vec<Cell>, Vec<Span>)>>>| -> bool {
        let Kind::Ball(centre, radius) = kinds[k] else { return false };
        boundary(k, boundaries);
        match &boundaries[k] {
            Some(Some((cells, spans))) => one_sided(faces[k], spans, cells, (centre, radius), e, level, delta),
            _ => false,
        }
    };
    for i in 0..faces.len() {
        for j in 0..i {
            let apart = match (&kinds[i], &kinds[j]) {
                (&Kind::Ball(a, ra), &Kind::Ball(b, rb)) => {
                    let d = a.sub(b).length();
                    if (d <= slack && (ra - rb).abs() <= slack) || d > ra + rb + band || d + band < (ra - rb).abs() {
                        true
                    } else {
                        // Spheres that cross: one face must keep to one side of
                        // the other's sphere, read on its own sphere as the cap
                        // plane `(x − c)·e = h0` ([`one_sided`]).
                        let toward = |from: Vec3, to: Vec3| to.sub(from).normalized().ok();
                        // The cap plane's height on a sphere of radius `r` facing one of `other_r`.
                        let cap = |r: f64, other_r: f64| (d * d + r * r - other_r * other_r) / (2.0 * d);
                        let (Some(eab), Some(eba)) = (toward(a, b), toward(b, a)) else { decline!() };
                        ball_side(i, eab, cap(ra, rb), depth * rb / d, &mut boundaries)
                            || ball_side(j, eba, cap(rb, ra), depth * ra / d, &mut boundaries)
                    }
                }
                (&Kind::Ball(c, r), Kind::Plane(plane)) | (Kind::Plane(plane), &Kind::Ball(c, r)) => {
                    let k = if matches!(kinds[i], Kind::Ball(..)) { i } else { j };
                    let (normal, origin) = (plane.normal(), plane.at_origin());
                    let level = normal.dot(origin.sub(c));
                    // A plane clear of the sphere, or the ball face to one side of it.
                    level.abs() > r + band || ball_side(k, normal, level, depth, &mut boundaries)
                }
                // (Coplanar faces pass here too: their heights are within the depth.)
                (Kind::Plane(p), Kind::Plane(q)) => plane_side(faces[i], p, q, depth) || plane_side(faces[j], q, p, depth),
            };
            if !apart {
                decline!();
            }
        }
    }
    // Well formed (ids, loops, edge uses, curve ends on their vertices: the
    // structural checks, without re-reading every pcurve against its edge,
    // which the proof does not use); a scoped subset's own open edges and
    // genus are the cut the acceptance made, which the scan never reads.
    let structure = solid.validate_scoped(&Default::default());
    if structure.iter().any(|issue| !matches!(issue.kind, crate::IssueKind::OpenEdge | crate::IssueKind::GenusMismatch)) {
        decline!();
    }
    // The trims the proof and the classification read: every loop's pcurves
    // chained end to start in (u, v), up to whole turns of u.
    if !faces.iter().all(|face| loops_closed_in_uv(face)) {
        decline!();
    }
    true
}

/// Whether each loop's pcurves meet end to start, to `1e-9` of the domain,
/// a step of a whole u period allowed (a ball's u is periodic).
fn loops_closed_in_uv(face: &FaceRecord) -> bool {
    let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else {
        return false;
    };
    let (period, size) = (u1 - u0, (u1 - u0).max(v1 - v0));
    let tolerance = 1e-9 * size;
    face.loops.iter().all(|loop_record| {
        let ends: Option<Vec<(Vec3, Vec3)>> = loop_record
            .coedges
            .iter()
            .map(|coedge| {
                let [q0, q1] = coedge.pcurve.domain().ok()?;
                Some((coedge.pcurve.evaluate(q0).ok()?, coedge.pcurve.evaluate(q1).ok()?))
            })
            .collect();
        let Some(ends) = ends else { return false };
        (0..ends.len()).all(|k| {
            let (end, start) = (ends[k].1, ends[(k + 1) % ends.len()].0);
            let du = end.x - start.x;
            let du = du - (du / period).round() * period;
            du.abs() <= tolerance && (end.y - start.y).abs() <= tolerance
        })
    })
}

/// `C(n, k)` for `k = 0..=n`.
fn binomials(n: usize) -> Vec<f64> {
    let mut row = vec![1.0; n + 1];
    for k in 1..=n {
        row[k] = row[k - 1] * (n - k + 1) as f64 / k as f64;
    }
    row
}

/// What a face of the body is, for this certificate.
enum Kind {
    Ball(Vec3, f64),
    Plane(super::analytic_body::Plane),
}

/// Whether plane face `face` (on `own`) lies within `depth` of one side of the
/// plane of `other` (its chart read as affine, which the recognition holds
/// to a slack far under the band, checked against every control's size): the face lies in the convex hull of its boundary, the
/// affine chart's image of its pcurves, each in the hull of its controls
/// (positive weights), so those controls' heights bound it.
fn plane_side(face: &FaceRecord, own: &super::analytic_body::Plane, other: &super::analytic_body::Plane, depth: f64) -> bool {
    let (normal, origin) = (other.normal(), other.at_origin());
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        for control in &coedge.pcurve.control_points {
            if !(control.w > 0.0) || ![control.x, control.y, control.w].iter().all(|v| v.is_finite()) {
                return false;
            }
            let height = normal.dot(own.at(control.x / control.w, control.y / control.w).sub(origin));
            lo = lo.min(height);
            hi = hi.max(height);
        }
    }
    lo.is_finite() && hi.is_finite() && (hi <= depth || lo >= -depth)
}

/// Bernstein coefficients of the product of two polynomials in Bernstein
/// form: scaled by their binomials the product is a convolution.
fn product(a: &[f64], b: &[f64]) -> Vec<f64> {
    let (p, q) = (a.len() - 1, b.len() - 1);
    let (bp, bq, bpq) = (binomials(p), binomials(q), binomials(p + q));
    let sa: Vec<f64> = a.iter().zip(&bp).map(|(x, c)| x * c).collect();
    let sb: Vec<f64> = b.iter().zip(&bq).map(|(x, c)| x * c).collect();
    let mut out = vec![0.0; p + q + 1];
    for (i, x) in sa.iter().enumerate() {
        for (j, y) in sb.iter().enumerate() {
            out[i + j] += x * y;
        }
    }
    for (slot, c) in out.iter_mut().zip(&bpq) {
        *slot /= c;
    }
    out
}

/// One Bezier cell of a native ball's net: its parameter box and its 3 x 3
/// homogeneous controls `[X, Y, Z, W]` (the net's own, as stored).
struct Cell {
    u: [f64; 2],
    v: [f64; 2],
    net: [[[f64; 4]; 3]; 3],
}

/// The cells of a biquadratic net whose interior knots are all double (C0
/// joins), as `make_sphere_surface` builds a ball: each knot-span pair's net
/// is a 3 x 3 subgrid of the stored one. `None` for any other layout.
fn cells(surface: &crate::NurbsSurface) -> Option<Vec<Cell>> {
    if surface.degree_u != 2 || surface.degree_v != 2 {
        return None;
    }
    let breaks = |knots: &[f64]| -> Option<Vec<f64>> {
        let n = knots.len();
        if n < 6 || knots[0] != knots[1] || knots[1] != knots[2] || knots[n - 1] != knots[n - 2] || knots[n - 2] != knots[n - 3] {
            return None;
        }
        let interior = &knots[3..n - 3];
        if interior.len() % 2 != 0 {
            return None;
        }
        let mut out = vec![knots[0]];
        for pair in interior.chunks(2) {
            if pair[0] != pair[1] || !(pair[0] > *out.last()?) {
                return None;
            }
            out.push(pair[0]);
        }
        if !(knots[n - 1] > *out.last()?) {
            return None;
        }
        out.push(knots[n - 1]);
        Some(out)
    };
    let (bu, bv) = (breaks(&surface.knots_u)?, breaks(&surface.knots_v)?);
    let controls = &surface.control_points;
    if controls.len() != 2 * (bu.len() - 1) + 1 || controls.iter().any(|row| row.len() != 2 * (bv.len() - 1) + 1) {
        return None;
    }
    let mut out = Vec::new();
    for i in 0..bu.len() - 1 {
        for j in 0..bv.len() - 1 {
            let mut net = [[[0.0; 4]; 3]; 3];
            for (k, row) in net.iter_mut().enumerate() {
                for (l, slot) in row.iter_mut().enumerate() {
                    let c = &controls[2 * i + k][2 * j + l];
                    if !(c.w > 0.0) || ![c.x, c.y, c.z, c.w].iter().all(|v| v.is_finite()) {
                        return None;
                    }
                    *slot = [c.x, c.y, c.z, c.w];
                }
            }
            out.push(Cell { u: [bu[i], bu[i + 1]], v: [bv[j], bv[j + 1]], net });
        }
    }
    Some(out)
}

/// How far, as a fraction of a cell's width, a pcurve span's hull may pass
/// the cell's side and still be read on that cell.
const OVERRUN: f64 = 1e-9;

/// The subdivision budget a span starts with ([`pcurve_sides`]), and the
/// level under which a span that fits a cell is no longer halved to tighten
/// its bound (crossing a cell side still splits it).
const LEVELS: usize = 16;
const REFINE_FLOOR: usize = LEVELS - 6;

/// A pcurve span in Bernstein form, homogeneous `[X, Y, W]`.
type Span = Vec<[f64; 3]>;

fn halves(span: &Span) -> (Span, Span) {
    split_at(span, 0.5)
}

/// A clamped pcurve's Bezier spans, homogeneous `[X, Y, W]`, by one pass of
/// knot refinement over its knots (each interior knot raised to the degree,
/// the standard decomposition, linear in the controls). `None` for a pcurve
/// that is not clamped, has non-positive weights or non-finite values.
fn pcurve_spans(curve: &crate::NurbsCurve) -> Option<Vec<Span>> {
    let p = curve.degree;
    let points: Vec<[f64; 3]> = curve.control_points.iter().map(|c| [c.x, c.y, c.w]).collect();
    let knots = &curve.knots;
    if p == 0 || points.iter().any(|h| !(h[2] > 0.0) || !h.iter().all(|v| v.is_finite())) || knots.len() != points.len() + p + 1 {
        return None;
    }
    let m = knots.len() - 1;
    if knots[..=p].iter().any(|k| *k != knots[0]) || knots[m - p..].iter().any(|k| *k != knots[m]) || !(knots[m] > knots[0]) {
        return None;
    }
    // Continuous only: an interior knot of multiplicity above the degree
    // breaks the curve, and the gap between its pieces is bounded by nothing.
    let interior = &knots[p + 1..m - p];
    if interior.chunk_by(|a, b| a == b).any(|run| run.len() > p) {
        return None;
    }
    let mut spans: Vec<Span> = vec![points[..=p].to_vec()];
    let (mut a, mut b) = (p, p + 1);
    let mut alphas = vec![0.0; p + 1];
    while b < m {
        let i = b;
        while b < m && knots[b + 1] == knots[b] {
            b += 1;
        }
        let multiplicity = b - i + 1;
        let last = spans.len() - 1;
        if multiplicity < p {
            let numerator = knots[b] - knots[a];
            for j in (multiplicity + 1..=p).rev() {
                alphas[j - multiplicity - 1] = numerator / (knots[a + j] - knots[a]);
            }
            let r = p - multiplicity;
            let mut next = vec![[0.0; 3]; p + 1];
            for j in 1..=r {
                let save = r - j;
                let start = multiplicity + j;
                for k in (start..=p).rev() {
                    let alpha = alphas[k - start];
                    for c in 0..3 {
                        spans[last][k][c] = alpha * spans[last][k][c] + (1.0 - alpha) * spans[last][k - 1][c];
                    }
                }
                if b < m {
                    next[save] = spans[last][p];
                }
            }
            if b < m {
                for k in p - multiplicity..=p {
                    next[k] = points[b - p + k];
                }
                spans.push(next);
            }
        } else if b < m {
            let mut next = vec![[0.0; 3]; p + 1];
            for k in 0..=p {
                next[k] = points[b - p + k];
            }
            spans.push(next);
        }
        if b < m {
            a = b;
            b += 1;
        }
    }
    spans.iter().all(|span| span.iter().all(|h| h[2] > 0.0 && h.iter().all(|v| v.is_finite()))).then_some(spans)
}

/// The span split at `tau` in (0, 1) by de Casteljau.
fn split_at(span: &Span, tau: f64) -> (Span, Span) {
    let mut work = span.clone();
    let n = work.len();
    let mut left = vec![work[0]];
    let mut right = vec![work[n - 1]];
    for level in 1..n {
        for k in 0..n - level {
            for c in 0..3 {
                work[k][c] = (1.0 - tau) * work[k][c] + tau * work[k + 1][c];
            }
        }
        left.push(work[0]);
        right.push(work[n - level - 1]);
    }
    right.reverse();
    (left, right)
}

/// Which side of the plane `{x : (x − anchor)·e = level}` the face's image of
/// a pcurve span keeps to, within `delta`: bit 0 when `(x − anchor)·e ≤
/// level + delta` all along it, bit 1 when `≥ level − delta`.
///
/// Exact, from the span's own controls: the span is halved until its (u, v)
/// control hull (positive weights: the curve lies in it) fits one cell of the
/// carrier, up to `levels` times. On a cell with local `s = (u − u0)/(u1 −
/// u0)` and `r` likewise, `s·W = (X − u0 W)/(u1 − u0) =: A`, `(1 − s)·W =: Ā`,
/// so the cell's Bernstein basis times `W²` is `Ā², 2AĀ, A²` (and in `r`
/// alike), and `F = Σ_kl b_k c_l ((N_kl − anchor w_kl)·e − (level ± delta) w_kl)`,
/// a polynomial of degree `4p` in Bernstein form, is `W⁴ · w(s, r) · ((x −
/// anchor)·e − level ∓ delta)` with both weights positive: its coefficients
/// bound its sign.
fn pcurve_sides(span: &Span, cells: &[Cell], anchor: Vec3, e: Vec3, level: f64, delta: f64, levels: usize) -> u8 {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for h in span {
        if !(h[2] > 0.0) || !h.iter().all(|v| v.is_finite()) {
            return 0;
        }
        let (u, v) = (h[0] / h[2], h[1] / h[2]);
        lo = [lo[0].min(u), lo[1].min(v)];
        hi = [hi[0].max(u), hi[1].max(v)];
    }
    let split = |levels: usize| -> u8 {
        if levels == 0 {
            return 0;
        }
        let (a, b) = halves(span);
        pcurve_sides(&a, cells, anchor, e, level, delta, levels - 1) & pcurve_sides(&b, cells, anchor, e, level, delta, levels - 1)
    };
    // A hull may pass a cell's side by `OVERRUN` of the cell's width; the
    // cell's polynomial then stands in for its neighbour's over that sliver,
    // which moves a point by at most the cell net's extent times it (taken
    // off the depth below).
    let fits = |c: &Cell| {
        let (su, sv) = (OVERRUN * (c.u[1] - c.u[0]), OVERRUN * (c.v[1] - c.v[0]));
        lo[0] >= c.u[0] - su && hi[0] <= c.u[1] + su && lo[1] >= c.v[0] - sv && hi[1] <= c.v[1] + sv
    };
    let Some(cell) = cells.iter().find(|c| fits(c)) else {
        // Split where the span crosses a cell side inside its hull, if it does;
        // halve otherwise.
        if levels == 0 {
            return split(levels);
        }
        let mut sides_u: Vec<f64> = cells.iter().flat_map(|c| c.u).filter(|k| *k > lo[0] && *k < hi[0]).collect();
        let mut sides_v: Vec<f64> = cells.iter().flat_map(|c| c.v).filter(|k| *k > lo[1] && *k < hi[1]).collect();
        sides_u.dedup();
        sides_v.dedup();
        let crossing = sides_u.first().map(|&k| (0usize, k)).or_else(|| sides_v.first().map(|&k| (1usize, k)));
        if let Some((axis, k)) = crossing {
            let f = |t: f64| -> f64 {
                let n = span.len() - 1;
                let mut work: Vec<f64> = span.iter().map(|h| h[axis] - k * h[2]).collect();
                for level in 1..=n {
                    for i in 0..=n - level {
                        work[i] = (1.0 - t) * work[i] + t * work[i + 1];
                    }
                }
                work[0]
            };
            let (f0, f1) = (f(0.0), f(1.0));
            if f0 != 0.0 && f1 != 0.0 && (f0 < 0.0) != (f1 < 0.0) {
                let (mut a, mut b) = (0.0f64, 1.0f64);
                for _ in 0..60 {
                    let m = 0.5 * (a + b);
                    if (f(m) < 0.0) == (f0 < 0.0) { a = m } else { b = m }
                }
                let tau = 0.5 * (a + b);
                if tau > 0.0 && tau < 1.0 {
                    let (left, right) = split_at(span, tau);
                    return pcurve_sides(&left, cells, anchor, e, level, delta, levels - 1)
                        & pcurve_sides(&right, cells, anchor, e, level, delta, levels - 1);
                }
            }
        }
        return split(levels);
    };
    // The overrun's worst displacement: the cell net's extent times it.
    let extent = {
        let (mut lo3, mut hi3) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for h in cell.net.iter().flatten() {
            for axis in 0..3 {
                let x = h[axis] / h[3];
                lo3[axis] = lo3[axis].min(x);
                hi3[axis] = hi3[axis].max(x);
            }
        }
        ((hi3[0] - lo3[0]).powi(2) + (hi3[1] - lo3[1]).powi(2) + (hi3[2] - lo3[2]).powi(2)).sqrt()
    };
    let delta = delta - 8.0 * OVERRUN * extent;
    if !(delta > 0.0) {
        return 0;
    }
    let (du, dv) = (cell.u[1] - cell.u[0], cell.v[1] - cell.v[0]);
    let x: Vec<f64> = span.iter().map(|h| h[0]).collect();
    let y: Vec<f64> = span.iter().map(|h| h[1]).collect();
    let w: Vec<f64> = span.iter().map(|h| h[2]).collect();
    let a: Vec<f64> = x.iter().zip(&w).map(|(x, w)| (x - cell.u[0] * w) / du).collect();
    let a_bar: Vec<f64> = x.iter().zip(&w).map(|(x, w)| (cell.u[1] * w - x) / du).collect();
    let b: Vec<f64> = y.iter().zip(&w).map(|(y, w)| (y - cell.v[0] * w) / dv).collect();
    let b_bar: Vec<f64> = y.iter().zip(&w).map(|(y, w)| (cell.v[1] * w - y) / dv).collect();
    let twice = |p: Vec<f64>| p.into_iter().map(|v| 2.0 * v).collect::<Vec<f64>>();
    let basis_s = [product(&a_bar, &a_bar), twice(product(&a, &a_bar)), product(&a, &a)];
    let basis_r = [product(&b_bar, &b_bar), twice(product(&b, &b_bar)), product(&b, &b)];
    let degree = basis_s[0].len() + basis_r[0].len() - 1;
    let (mut upper, mut lower) = (vec![0.0; degree], vec![0.0; degree]);
    // Σ_k b_k (Σ_l f_kl c_l): the inner sums first, three products per bound.
    for k in 0..3 {
        let (mut inner_upper, mut inner_lower) = (vec![0.0; basis_r[0].len()], vec![0.0; basis_r[0].len()]);
        for l in 0..3 {
            let [nx, ny, nz, nw] = cell.net[k][l];
            let along = (nx - anchor.x * nw) * e.x + (ny - anchor.y * nw) * e.y + (nz - anchor.z * nw) * e.z;
            let (fu, fl) = (along - (level + delta) * nw, along - (level - delta) * nw);
            for (index, c) in basis_r[l].iter().enumerate() {
                inner_upper[index] += fu * c;
                inner_lower[index] += fl * c;
            }
        }
        for (slot, v) in upper.iter_mut().zip(product(&basis_s[k], &inner_upper)) {
            *slot += v;
        }
        for (slot, v) in lower.iter_mut().zip(product(&basis_s[k], &inner_lower)) {
            *slot += v;
        }
    }
    if !upper.iter().chain(&lower).all(|v| v.is_finite()) {
        return 0;
    }
    let mut sides = 0u8;
    if upper.iter().all(|v| *v <= 0.0) {
        sides |= 1;
    }
    if lower.iter().all(|v| *v >= 0.0) {
        sides |= 2;
    }
    // Bernstein bounds tighten quadratically: a span the bound still splits
    // after a few halvings passes the plane in truth (or by a hair); stop.
    if sides == 0 && levels > REFINE_FLOOR { split(levels) } else { sides }
}

/// Whether `face`, on the sphere `own = (c_a, r_a)`, keeps to one side of
/// the sphere `other = (c_b, r_b)`, so closely that it reaches into or out of
/// the other ball by at most an eighth of the straddle band `band`.
///
/// On the sphere of `face`, with `e` the unit vector from `c_a` to `c_b` at
/// distance `d`, a point is inside the other ball exactly when `(x − c_a)·e >
/// h0 = (d² + r_a² − r_b²) / (2d)`, and reaches into it by about `d (h − h0) /
/// r_b`. So the face's region keeps out of the ball within the depth when its
/// boundary, the images of all its pcurves (seams and collapsed pole sides
/// included), never passes `h0 + δ`, `δ = band r_b / (8 d)` ([`pcurve_sides`]),
/// and the open cap `h > h0 + δ` of its sphere, connected and missed by that
/// boundary, lies outside the region: its deepest point, `c_a + r_a e`, is read
/// Outside by the trim classification (the chart is injective inside the face).
/// Likewise with the sides exchanged, at `c_a − r_a e`. A face that keeps to
/// one side of the other sphere does not straddle it by more than the depth,
/// so no crossing with a face on it can be confirmed.
fn one_sided(face: &FaceRecord, spans: &[Span], cells: &[Cell], own: (Vec3, f64), e: Vec3, level: f64, delta: f64) -> bool {
    let (centre_a, radius_a) = own;
    if !(delta > 0.0 && delta.is_finite() && level.is_finite()) {
        return false;
    }
    let mut sides = 3u8;
    for span in spans {
        sides &= pcurve_sides(span, cells, centre_a, e, level, delta, LEVELS);
        if sides == 0 {
            if std::env::var_os("BREP_DEBUG_DISJOINT_BALLS").is_some() {
                eprintln!("soundness.disjoint_balls one_sided: face {} has a pcurve span on both sides of the plane", face.id);
            }
            return false;
        }
    }
    // Points of the cap beyond the depth: its extreme point, and points
    // tilted from it by half the cap's angular radius four ways (any point of
    // the connected cap decides; the tilted ones avoid a seam or a pole the
    // extreme point may sit on).
    let mut probes: Vec<(Vec3, bool)> = Vec::new();
    for (bit, sign) in [(1u8, 1.0f64), (2u8, -1.0f64)] {
        if sides & bit == 0 {
            continue;
        }
        let axis = e.scale(sign);
        // The height along `axis` the cap starts at, and its angular radius.
        let start = sign * level + delta;
        let deep = radius_a - start > 4.0 * delta;
        probes.push((centre_a.add(axis.scale(radius_a)), deep));
        let Ok(x) = axis.perpendicular() else { continue };
        let y = axis.cross(x);
        let half = 0.5 * (start / radius_a).clamp(-1.0, 1.0).acos();
        for direction in [x, x.scale(-1.0), y, y.scale(-1.0)] {
            let point = centre_a.add(axis.scale(radius_a * half.cos()).add(direction.scale(radius_a * half.sin())));
            let height = point.sub(centre_a).dot(axis);
            probes.push((point, height - start > 4.0 * delta));
        }
    }
    let debug = std::env::var_os("BREP_DEBUG_DISJOINT_BALLS").is_some();
    probes.iter().any(|&(probe, deep)| {
        if debug {
            eprintln!("soundness.disjoint_balls one_sided: face {} sides {sides} probe deep {deep}", face.id);
        }
        if !deep {
            return false;
        }
        let Ok(projection) = crate::project_point_to_surface(&face.surface, probe) else {
            return false;
        };
        if !(projection.distance <= 1e-9 * (1.0 + radius_a)) {
            return false;
        }
        let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else {
            return false;
        };
        let margin = 1e-6;
        if projection.v <= v0 + margin * (v1 - v0) || projection.v >= v1 - margin * (v1 - v0) || projection.u <= u0 + margin * (u1 - u0) || projection.u >= u1 - margin * (u1 - u0) {
            return false;
        }
        let tolerance = crate::classification::surface_uv_band(&face.surface, projection.u, projection.v, delta);
        let class = parameter_point_in_face(face, Vec2 { x: projection.u, y: projection.v }, tolerance);
        if debug {
            eprintln!("soundness.disjoint_balls one_sided: face {} probe uv ({:.6}, {:.6}) class {:?}", face.id, projection.u, projection.v, class.as_ref().map(|c| format!("{c:?}")));
        }
        matches!(class, Ok(PolygonClass::Outside))
    })
}

