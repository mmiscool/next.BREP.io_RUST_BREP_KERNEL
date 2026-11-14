//! A rounded offset read back against its own definition where it is most
//! often lost (`BREP_OFFSET_RIM_WITNESS`, default on; `=0` returns the body
//! unread).
//!
//! The parallel body is a set: grown by `d`, every point within `d` of the
//! source; shrunk by `d`, every point of the source at least `d` from its
//! boundary. Where the offset passes the radius `r` of a circular edge, the
//! ball rolled about that edge reaches the edge's own axis, and the body keeps
//! a piece of a spindle torus there: a dimple over a closed bore or pocket, a
//! bump under a pin swallowed by a shrink, the same across the arc of a
//! concave fillet. The pipeline loses the feature's faces once their offset
//! radius reaches zero and with them the edge the round would have rolled, so
//! the body comes out filled (or bare) there and still validates: a blind bore
//! of radius 1 grown by 2 read 3038.540974 for 3038.406558; a spherical pocket
//! of radius 2 grown by 3, 4310.876698 for 4309.409353; a pin of radius 1
//! shrunk by 1.5 was the plain shrunk disc, 132.732290 for 132.915708; the
//! L-block with its inside corner rounded to 1, grown by 2, 3485.037099 for
//! 3484.726277.
//!
//! [`super::bore_dimples`] carves the dimple where it can prove it. This reads
//! every result: on the axis of each circular edge of radius under `|d|`,
//! halfway between where the rim's torus crosses the axis and the offset
//! plane, the point's distance to the source's boundary (trimmed: a face's own
//! region, or its edges) says whether the point belongs to the parallel body,
//! and the result is asked the same question. A disagreement refuses by name;
//! a point too near the bar to call is left alone.
use crate::classification::PolygonClass;
use crate::topology::BrepSolid;
use crate::{classify_point, parameter_point_in_face, project_point_to_surface, KernelRefusal, KernelStage, PointClass, Vec2, Vec3};

pub(super) fn rim_witness_enabled() -> bool {
    switch!("BREP_OFFSET_RIM_WITNESS", true)
}

/// The distance from `point` to the boundary of `source`: over every face, the
/// foot on its surface where that foot lies in the face, and over every edge,
/// its nearest of 129 stations. `None` when a face or an edge cannot be read.
fn boundary_distance(source: &BrepSolid, point: Vec3) -> Option<f64> {
    let mut best = f64::INFINITY;
    for face in source.shells.iter().flat_map(|shell| &shell.faces) {
        let foot = project_point_to_surface(&face.surface, point).ok()?;
        if foot.distance < best
            && !matches!(parameter_point_in_face(face, Vec2 { x: foot.u, y: foot.v }, 1e-9).ok()?, PolygonClass::Outside)
        {
            best = foot.distance;
        }
    }
    for edge in source.edges.iter().filter(|edge| !edge.degenerate) {
        for k in 0..=128 {
            let at = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 128.0).ok()?;
            best = best.min(at.sub(point).length());
        }
    }
    best.is_finite().then_some(best)
}

/// The circle an edge lies on, as (centre, unit normal, radius), read at nine
/// stations; `None` for any other curve.
fn circle_of(edge: &crate::topology::EdgeRecord, scale: f64) -> Option<(Vec3, Vec3, f64)> {
    let at = |k: usize| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 8.0).ok();
    let (a, b, c) = (at(0)?, at(3)?, at(6)?);
    let (ab, ac) = (b.sub(a), c.sub(a));
    let normal = ab.cross(ac);
    let twice = normal.length_squared();
    if twice <= 1e-18 * scale.powi(4) {
        return None;
    }
    // The circumcentre of a, b, c.
    let centre = a.add(
        normal.cross(ab).scale(ac.length_squared()).add(ac.cross(normal).scale(ab.length_squared())).scale(0.5 / twice),
    );
    let radius = a.sub(centre).length();
    let normal = normal.scale(1.0 / twice.sqrt());
    for k in 0..=8 {
        let d = at(k)?.sub(centre);
        if (d.length() - radius).abs() > 1e-7 * scale || d.dot(normal).abs() > 1e-7 * scale {
            return None;
        }
    }
    Some((centre, normal, radius))
}

/// The refusal for `result` (the rounded offset of `source` by `distance`)
/// where it disagrees with the parallel body's definition on the axis of a
/// circular edge the offset has passed, or `None`.
pub(super) fn rim_witness_refusal(source: &BrepSolid, result: &BrepSolid, distance: f64) -> Option<KernelRefusal> {
    if !rim_witness_enabled() || distance == 0.0 {
        return None;
    }
    let reach = distance.abs();
    let scale = crate::solid_scale(source).max(1.0);
    let class = |solid: &BrepSolid, point: Vec3| classify_point(point, solid, 1e-9 * scale).ok().map(|c| c.class);
    let mut seen: Vec<(Vec3, Vec3, f64)> = Vec::new();
    for edge in source.edges.iter().filter(|edge| !edge.degenerate) {
        let Some((centre, normal, radius)) = circle_of(edge, scale) else { continue };
        if !(radius > 1e-9 * scale && radius < reach * (1.0 - 1e-6)) {
            continue;
        }
        if seen.iter().any(|(c, n, r)| c.sub(centre).length() <= 1e-7 * scale && n.cross(normal).length() <= 1e-7 && (r - radius).abs() <= 1e-7 * scale) {
            continue;
        }
        seen.push((centre, normal, radius));
        let low = (reach * reach - radius * radius).sqrt();
        for side in [1.0, -1.0] {
            let witness = centre.add(normal.scale(side * 0.5 * (low + reach)));
            let (Some(in_source), Some(in_result)) = (class(source, witness), class(result, witness)) else { continue };
            // Growing, only a point outside the source is in question; shrinking, only one inside it.
            let wanted = if distance > 0.0 { PointClass::Out } else { PointClass::In };
            if in_source != wanted {
                continue;
            }
            let Some(gap) = boundary_distance(source, witness) else { continue };
            let belongs = if gap < reach * (1.0 - 1e-3) {
                distance > 0.0
            } else if gap > reach * (1.0 + 1e-3) {
                distance < 0.0
            } else {
                continue;
            };
            let disagrees = match in_result {
                PointClass::In => !belongs,
                PointClass::Out => belongs,
                _ => false,
            };
            if disagrees {
                return Some(KernelRefusal::unsupported(
                    KernelStage::Validate,
                    "offset_rim_spindle",
                    format!(
                        "offset_solid: the offset by {distance} passes the radius {radius:.6} of a circular edge about \
                         ({:.6}, {:.6}, {:.6}), and the body built does not keep the piece of the rolled ball's torus that \
                         reaches the edge's axis: the point ({:.6}, {:.6}, {:.6}) on that axis is {gap:.6} from the source's \
                         boundary, so it {} the parallel body, and the body built says otherwise — refusing rather than \
                         returning it",
                        centre.x,
                        centre.y,
                        centre.z,
                        witness.x,
                        witness.y,
                        witness.z,
                        if belongs { "is in" } else { "is outside" },
                    ),
                ));
            }
        }
    }
    None
}
