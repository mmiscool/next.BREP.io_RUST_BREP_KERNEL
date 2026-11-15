//! Planar termination of a stripe on curved carriers.
use super::*;
use crate::{fit, NurbsSurface, Vec4};

pub(super) struct Section {
    pub cr: f64,
    pub cs: f64,
    pub first: Vec3,
    pub second: Vec3,
    pub curve: NurbsCurve,
    pub pcurve: NurbsCurve,
    /// `None` on a side the stopped-end rollover closes with its own leg chain.
    pub legs: [Option<(NurbsCurve, NurbsCurve)>; 2],
}

fn refusal() -> KernelRefusal {
    KernelRefusal::non_convergence(
        KernelStage::Intersect,
        "cap_plane_section",
        "blend network: could not trace the cap plane on its carriers",
    )
}

fn crossing(
    curve: &NurbsCurve,
    origin: Vec3,
    normal: Vec3,
    seed: f64,
) -> Result<f64, KernelRefusal> {
    let [lo, hi] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let mut t = seed;
    for _ in 0..40 {
        let d = curve
            .derivatives(t, 1)
            .or_refuse(KernelStage::Refine, "derivatives")?;
        let distance = d[0].sub(origin).dot(normal);
        if distance.abs() < 1e-11 {
            return Ok(t);
        }
        let slope = d[1].dot(normal);
        if slope.abs() < 1e-14 {
            break;
        }
        t -= distance / slope;
        if !(t > lo && t < hi) {
            break;
        }
    }
    Err(refusal())
}

/// `carriers`: per side, the face the leg is traced on when it is NOT the
/// mate — the neighbour a stopped-end rollover's track rolled onto; its
/// rim and sharp-vertex parameter points are read by projection.
pub(super) fn plane_section(
    stripe: &Stripe,
    rows: &FittedRows,
    origin: Vec3,
    normal: Vec3,
    station: f64,
    at_start: bool,
    carriers: [Option<&FaceRecord>; 2],
) -> Result<Section, KernelRefusal> {
    let cr = crossing(&rows.cr, origin, normal, station)?;
    let cs = crossing(&rows.cs, origin, normal, station)?;
    let first = rows
        .cr
        .evaluate(cr)
        .or_refuse(KernelStage::Refine, "evaluate")?;
    let second = rows
        .cs
        .evaluate(cs)
        .or_refuse(KernelStage::Refine, "evaluate")?;
    let u = second
        .sub(first)
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    let v = normal
        .cross(u)
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    let reach = first.sub(origin).length().max(second.sub(origin).length()) * 4.0;
    let mut plane = stripe.first.face.clone();
    plane.surface = crate::make_plane(
        origin.sub(u.scale(reach)).sub(v.scale(reach)),
        u,
        v,
        2.0 * reach,
        2.0 * reach,
    )
    .or_refuse(KernelStage::Refine, "make_plane")?;
    let (curve, pcurve, _) = transverse_curve(rows, &plane, cr, cs)?;
    let leg = |mate: &BlendMate, carrier: Option<&FaceRecord>, row: &NurbsCurve, t, point: Vec3| {
        if carrier.is_some() {
            // A rolled side's leg is a chain over the neighbours (`leg_chain`).
            return Ok(None);
        }
        let start = row.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let edge_t = if at_start {
            stripe.edge.t0
        } else {
            stripe.edge.t1
        };
        let end = edge_uv_on_face(mate.coedge, stripe.edge, edge_t)?;
        plane_leg(
            &mate.face.surface,
            origin,
            normal,
            point,
            [start.x, start.y],
            end,
        ).map(Some)
    };
    let legs = [
        leg(&stripe.first, carriers[0], &rows.cr_pcurve, cr, first)?,
        leg(&stripe.second, carriers[1], &rows.cs_pcurve, cs, second)?,
    ];
    Ok(Section {
        cr,
        cs,
        first,
        second,
        curve,
        pcurve,
        legs,
    })
}

/// Trace the plane/carrier intersection from the rail to the sharp vertex.
/// The second equation advances along the leg's chord; the UV interpolation
/// is only a seed, never a substitute for the intersection.
fn plane_leg(
    surface: &NurbsSurface,
    origin: Vec3,
    normal: Vec3,
    rim: Vec3,
    start: [f64; 2],
    end: [f64; 2],
) -> Result<(NurbsCurve, NurbsCurve), KernelRefusal> {
    plane_leg_between(surface, origin, normal, rim, origin, start, end)
}

/// [`plane_leg`] between two points of the plane on the carrier: `from` to
/// `to`, both preserved exactly as the curve's ends.
fn plane_leg_between(
    surface: &NurbsSurface,
    origin: Vec3,
    normal: Vec3,
    from: Vec3,
    to: Vec3,
    start: [f64; 2],
    end: [f64; 2],
) -> Result<(NurbsCurve, NurbsCurve), KernelRefusal> {
    let rim = from;
    let chord = to.sub(rim);
    let length = chord.length();
    let direction = chord
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    let mut points = Vec::new();
    let mut uvs = Vec::new();
    let mut ts = Vec::new();
    for i in 0..=32 {
        let t = i as f64 / 32.0;
        let mut uv = [
            start[0] * (1.0 - t) + end[0] * t,
            start[1] * (1.0 - t) + end[1] * t,
        ];
        let mut converged = false;
        for _ in 0..40 {
            let d = surface
                .derivatives_extended(uv[0], uv[1], 1)
                .or_refuse(KernelStage::Refine, "derivatives_extended")?;
            let p = d[0][0];
            let f = [
                p.sub(origin).dot(normal),
                p.sub(rim).dot(direction) - t * length,
            ];
            if f[0].abs().max(f[1].abs()) < 1e-11 {
                converged = true;
                break;
            }
            let a = d[1][0].dot(normal);
            let b = d[0][1].dot(normal);
            let c = d[1][0].dot(direction);
            let d = d[0][1].dot(direction);
            let det = a * d - b * c;
            if det.abs() < 1e-16 {
                break;
            }
            uv[0] -= (d * f[0] - b * f[1]) / det;
            uv[1] -= (a * f[1] - c * f[0]) / det;
        }
        if !converged {
            return Err(refusal());
        }
        let point = surface
            .evaluate_extended(uv[0], uv[1])
            .or_refuse(KernelStage::Refine, "evaluate_extended")?;
        // Preserve the rail's shared endpoint; its carrier discrepancy is
        // checked with the complete sewn body's pcurve consistency policy.
        points.push(Vec4::from_point(
            if i == 0 {
                rim
            } else if i == 32 {
                to
            } else {
                point
            },
            1.0,
        ));
        uvs.push(Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0));
        ts.push(t);
    }
    Ok((
        fit::interpolate_homogeneous(&points, 3, &ts)
            .or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
        fit::interpolate_homogeneous(&uvs, 3, &ts)
            .or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
    ))
}

/// The parameters where `curve` (over `[t0, t1]`) meets the plane through
/// `origin` with `normal`: bracketed on 64 samples, refined by Newton.
fn plane_roots(curve: &NurbsCurve, t0: f64, t1: f64, origin: Vec3, normal: Vec3) -> Result<Vec<f64>, KernelRefusal> {
    const SAMPLES: usize = 64;
    let mut roots = Vec::new();
    let mut previous: Option<(f64, f64)> = None;
    for k in 0..=SAMPLES {
        let t = t0 + (t1 - t0) * k as f64 / SAMPLES as f64;
        let value = curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin).dot(normal);
        if let Some((pt, pv)) = previous {
            if pv == 0.0 || (pv < 0.0) != (value < 0.0) {
                // Newton from the bracket's linear estimate, kept inside it.
                let mut lo = pt;
                let mut hi = t;
                let mut root = pt + (t - pt) * (pv / (pv - value)).clamp(0.0, 1.0);
                for _ in 0..40 {
                    let d = curve.derivatives(root, 1).or_refuse(KernelStage::Refine, "derivatives")?;
                    let f = d[0].sub(origin).dot(normal);
                    if f.abs() < 1e-12 {
                        break;
                    }
                    if (f < 0.0) == (pv < 0.0) { lo = root; } else { hi = root; }
                    let slope = d[1].dot(normal);
                    let next = if slope.abs() > 1e-14 { root - f / slope } else { 0.5 * (lo + hi) };
                    root = if next > lo && next < hi { next } else { 0.5 * (lo + hi) };
                }
                roots.push(root);
            }
        }
        previous = Some((t, value));
    }
    Ok(roots)
}

/// A stopped-end cap's leg from the rim on `carrier_id` to the sharp vertex
/// as a chain of plane/carrier intersections, one piece per face: while the
/// carrier does not hold the sharp vertex, the leg leaves it where the cap
/// plane meets the carrier's boundary ahead of the trace, that edge is split
/// there (both faces' coedges with it), and the chain continues on the face
/// across. Every piece's edge and vertex is committed to `result`.
#[allow(clippy::too_many_arguments)]
pub(super) fn leg_chain(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    carrier_id: u64,
    rim_point: Vec3,
    rim_vertex: u64,
    sharp_point: Vec3,
    sharp_vertex: u64,
    origin: Vec3,
    normal: Vec3,
) -> Result<Vec<crate::blend::rollover::LegPiece>, KernelRefusal> {
    let what = "stopped_end_rollover";
    let not_built = |message: String| KernelRefusal::unsupported(KernelStage::Refine, what, format!("blend stopped-end rollover: {message}"));
    let band = crate::KernelTolerances::for_solid(result, 1e-7).model.max(1e-9);
    let mut pieces = Vec::new();
    let (mut face_id, mut current_point, mut current_vertex) = (carrier_id, rim_point, rim_vertex);
    for _hop in 0..4 {
        let face = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_id)
            .cloned()
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, what, format!("blend stopped-end rollover: leg carrier face {face_id} missing")))?;
        let edge_of = |id: u64| result.edges.iter().find(|edge| edge.id == id).cloned();
        let holds_sharp = face.loops.iter().flat_map(|lp| &lp.coedges).any(|coedge| {
            edge_of(coedge.edge_id).is_some_and(|edge| edge.start_vertex_id == sharp_vertex || edge.end_vertex_id == sharp_vertex)
        });
        let start = crate::project_point_to_surface(&face.surface, current_point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        if holds_sharp {
            let end = crate::project_point_to_surface(&face.surface, sharp_point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            let (curve, pcurve) = plane_leg_between(&face.surface, origin, normal, current_point, sharp_point, [start.u, start.v], [end.u, end.v])?;
            let id = take_id();
            result.edges.push(EdgeRecord { id, curve, t0: 0.0, t1: 1.0, start_vertex_id: current_vertex, end_vertex_id: sharp_vertex, degenerate: false, name: None });
            pieces.push(crate::blend::rollover::LegPiece { edge_id: id, face_id, from: current_vertex, to: sharp_vertex, pcurve });
            return Ok(pieces);
        }
        // The exit: the nearest plane crossing of the face's boundary ahead
        // of the trace (toward the sharp point).
        let ahead = sharp_point.sub(current_point);
        let mut exit: Option<(f64, u64, f64, Vec3)> = None;
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            let Some(edge) = edge_of(coedge.edge_id) else { continue };
            for t in plane_roots(&edge.curve, edge.t0, edge.t1, origin, normal)? {
                let point = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
                let distance = point.sub(current_point).length();
                if distance <= band || point.sub(current_point).dot(ahead) <= 0.0 {
                    continue;
                }
                if exit.is_none_or(|(best, _, _, _)| distance < best) {
                    exit = Some((distance, edge.id, t, point));
                }
            }
        }
        let Some((_, exit_edge_id, exit_t, exit_point)) = exit else {
            return Err(not_built(format!("the cap plane leaves face {face_id} across no boundary edge on its way to vertex {sharp_vertex}")));
        };
        let exit_edge = edge_of(exit_edge_id).expect("the exit edge");
        // The face across the exit edge, read before the split renames it.
        let across = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|other| other.id != face_id && other.loops.iter().flat_map(|lp| &lp.coedges).any(|coedge| coedge.edge_id == exit_edge_id))
            .map(|other| other.id)
            .ok_or_else(|| not_built(format!("edge {exit_edge_id} has no face across from face {face_id}")))?;
        // The exit vertex: an existing end of the edge within the band, else
        // a fresh vertex and the edge split there.
        let vertex_point = |id: u64| result.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
        let exit_vertex = if vertex_point(exit_edge.start_vertex_id).is_some_and(|p| p.sub(exit_point).length() <= band) {
            exit_edge.start_vertex_id
        } else if vertex_point(exit_edge.end_vertex_id).is_some_and(|p| p.sub(exit_point).length() <= band) {
            exit_edge.end_vertex_id
        } else {
            let id = take_id();
            result.vertices.push(VertexRecord { id, point: exit_point });
            crate::blend::rollover_surgery::split_edge_at_vertices(result, exit_edge_id, &[(id, exit_point)], take_id)?;
            id
        };
        let _ = exit_t;
        let end = crate::project_point_to_surface(&face.surface, exit_point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        let (curve, pcurve) = plane_leg_between(&face.surface, origin, normal, current_point, exit_point, [start.u, start.v], [end.u, end.v])?;
        let id = take_id();
        result.edges.push(EdgeRecord { id, curve, t0: 0.0, t1: 1.0, start_vertex_id: current_vertex, end_vertex_id: exit_vertex, degenerate: false, name: None });
        pieces.push(crate::blend::rollover::LegPiece { edge_id: id, face_id, from: current_vertex, to: exit_vertex, pcurve });
        face_id = across;
        current_point = exit_point;
        current_vertex = exit_vertex;
    }
    Err(not_built(format!("the cap's leg did not reach vertex {sharp_vertex} within four faces")))
}
