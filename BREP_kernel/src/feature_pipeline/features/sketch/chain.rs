use super::*;
use super::geometry::{ellipse_point, sample_bezier_polyline, SegKind, Segment};

/// A maximal endpoint-connected run of segments in head-to-tail order
/// (`(index, reversed)` into the segment list). `closed` when the run's tail
/// returned to its head.
pub(super) struct Chain {
    pub(super) entries: Vec<(usize, bool)>,
    pub(super) closed: bool,
}

/// Chain segments into maximal runs by near-coincident endpoints ([`close`]),
/// growing each seed forward from its tail then backward from its head — a
/// greedy chainer, except closure is checked
/// BEFORE extending so a closed loop never overshoots into an attached open
/// chain. Every segment lands in exactly one chain; branching junctions simply
/// end a run (the remaining branch seeds its own chain), so mixed sketches
/// never hard-error here.
pub(super) fn chain_segments(segments: &[Segment]) -> Vec<Chain> {
    let count = segments.len();
    let mut used = vec![false; count];
    let mut chains = Vec::new();
    while let Some(seed) = used.iter().position(|&u| !u) {
        used[seed] = true;
        let mut entries: Vec<(usize, bool)> = vec![(seed, false)];
        let mut head = segments[seed].a;
        let mut tail = segments[seed].b;
        let mut closed = false;
        loop {
            if close(tail, head) {
                closed = true;
                break;
            }
            if let Some((index, reversed, far)) = attach(segments, &used, tail) {
                used[index] = true;
                entries.push((index, reversed));
                tail = far;
                continue;
            }
            if let Some((index, reversed, far)) = attach(segments, &used, head) {
                used[index] = true;
                // Walking head-ward: `attach` orients tail-ward, so flip. A
                // segment whose natural END touches the head flows INTO it
                // un-reversed (`b≈head → (i,false), head=a`); one whose natural
                // START touches runs reversed (`a≈head → (i,true), head=b`).
                entries.insert(0, (index, !reversed));
                head = far;
                continue;
            }
            break;
        }
        chains.push(Chain { entries, closed });
    }
    chains
}

/// The first unused segment with an endpoint near `cursor`, oriented to walk
/// AWAY from it: `(index, reversed, far_end)`. Index-order scan, deterministic.
fn attach(segments: &[Segment], used: &[bool], cursor: [f64; 2]) -> Option<(usize, bool, [f64; 2])> {
    for (index, segment) in segments.iter().enumerate() {
        if used[index] {
            continue;
        }
        if close(segment.a, cursor) {
            return Some((index, false, segment.b));
        }
        if close(segment.b, cursor) {
            return Some((index, true, segment.a));
        }
    }
    None
}

/// Total geometric length of a planned chain (line chord / arc length), for
/// picking the LONGEST open chain as the sketch's path.
pub(super) fn plan_length(plan: &[(usize, bool)], segments: &[Segment]) -> f64 {
    plan.iter()
        .map(|&(index, _)| {
            let segment = &segments[index];
            match &segment.kind {
                SegKind::Line => {
                    (segment.b[0] - segment.a[0]).hypot(segment.b[1] - segment.a[1])
                }
                SegKind::Arc { radius, sweep, .. } => radius * sweep,
                SegKind::Bezier { controls } => sample_bezier_polyline(controls, 16)
                    .windows(2)
                    .map(|pair| (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]))
                    .sum(),
                SegKind::EllipseHalf {
                    center,
                    major,
                    minor,
                    upper,
                } => {
                    // Polyline approximation over the half's angle span (an
                    // ellipse half never chains into an open path, but the
                    // length must still be well-defined).
                    let base = if *upper { PI } else { 0.0 };
                    (0..16)
                        .map(|step| {
                            let t0 = base + PI * (step as f64 / 16.0);
                            let t1 = base + PI * ((step + 1) as f64 / 16.0);
                            let p0 = ellipse_point(*center, *major, *minor, t0);
                            let p1 = ellipse_point(*center, *major, *minor, t1);
                            (p1[0] - p0[0]).hypot(p1[1] - p0[1])
                        })
                        .sum()
                }
            }
        })
        .sum()
}

/// Emit a planned chain's world curves in order, welded exactly head-to-tail
/// (and end-to-start when `closed`).
pub(super) fn emit_chain(
    plan: &[(usize, bool)],
    segments: &[Segment],
    frame: &Frame,
    closed: bool,
) -> Result<Vec<NurbsCurve>, String> {
    let mut curves = Vec::with_capacity(plan.len());
    for &(index, reversed) in plan {
        curves.push(segments[index].to_curve(frame, reversed)?);
    }
    weld_chain(&mut curves, closed);
    Ok(curves)
}

/// Weld a chained curve list EXACTLY head-to-tail (and last-end→first-start when
/// `closed`) by translating endpoint control points only — the chainer joins
/// within [`JOIN_TOL`] (1e-5, the established tolerance) but the kernel demands 1e-6
/// closure (`extrude_profile_brep`). Mirrors the established endpoint snap
/// (`pts[0] = [sa.x, sa.y]`): exact for lines; for arcs only the endpoint
/// control point moves (weights preserved) — a ≤ 1e-5 shape nudge.
pub(super) fn weld_chain(curves: &mut [NurbsCurve], closed: bool) {
    for index in 1..curves.len() {
        let target = curve_endpoint(&curves[index - 1], false);
        snap_curve_endpoint(&mut curves[index], target, true);
    }
    if closed && curves.len() >= 2 {
        let target = curve_endpoint(&curves[0], true);
        let last = curves.len() - 1;
        snap_curve_endpoint(&mut curves[last], target, false);
    }
}

/// A clamped curve's exact start/end point (first/last dehomogenized control
/// point — every profile/path curve here comes from `make_line`/`make_arc`,
/// which are clamped).
fn curve_endpoint(curve: &NurbsCurve, at_start: bool) -> Vec3 {
    let point = if at_start {
        curve.control_points.first()
    } else {
        curve.control_points.last()
    }
    .copied()
    .expect("profile curves have control points");
    Vec3::new(point.x / point.w, point.y / point.w, point.z / point.w)
}

/// Move a clamped curve's start/end to `target` by rewriting that endpoint
/// control point (weight preserved; homogeneous coordinates).
fn snap_curve_endpoint(curve: &mut NurbsCurve, target: Vec3, at_start: bool) {
    let index = if at_start {
        0
    } else {
        curve.control_points.len() - 1
    };
    let weight = curve.control_points[index].w;
    curve.control_points[index] = Vec4::from_point(target, weight);
}

/// Endpoint coincidence within [`JOIN_TOL`], PER AXIS — the per-axis metric,
/// which admits a
/// diagonal gap up to √2·eps that a euclidean test would reject.
fn close(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() <= JOIN_TOL && (a[1] - b[1]).abs() <= JOIN_TOL
}
