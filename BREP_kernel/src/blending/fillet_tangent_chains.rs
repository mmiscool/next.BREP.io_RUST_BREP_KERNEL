//! A fillet of edges and every edge their G1 tangent chains continue into:
//! the fillet feature's "Follow tangent" selection, as one public call; and
//! the fillet with the caller's options (follow tangents, refuse a pinch).
use crate::{fillet_edges, BrepSolid, KernelRefusal, KernelStage, Vec3};

/// [`fillet_edges`] of `edge_points`' edges and every unselected edge on the
/// tangent chains through them (closed loops included: picking one straight
/// edge of a prism whose upright corners are rounded rounds its whole top
/// loop). The chains are walked as the fillet feature walks them
/// (`tangent_chain_expansion`: from each end vertex the single edge whose
/// tangent continues the chain within 1e-6, stopping at a sharp or ambiguous
/// junction). `edge_names`, when given, names the picked edges; the edges the
/// chains add carry their own names.
pub fn fillet_edges_tangent_chains(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let seeds = edge_points
        .iter()
        .map(|point| brep_blend::fillet::edges::resolve_edge_by_point(solid, *point))
        .collect::<Result<Vec<u64>, KernelRefusal>>()?;
    let added = crate::feature_pipeline::features::common::tangent_chain_expansion(solid, &seeds, &seeds)
        .map_err(|error| KernelRefusal::internal(KernelStage::Collect, "tangent_chain", error))?;
    if added.is_empty() {
        return fillet_edges(solid, edge_points, edge_names, radius, false, name);
    }
    let mut points = edge_points.to_vec();
    points.extend(added.iter().map(|(_, point, _)| *point));
    let names = edge_names.map(|names| {
        let mut all = names.to_vec();
        all.extend(added.iter().map(|(_, _, name)| name.clone()));
        all
    });
    fillet_edges(solid, &points, names.as_deref(), radius, false, name)
}

/// How [`fillet_edges_with_options`] reads a selection.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FilletOptions {
    /// Also fillet every edge the picked edges' tangent chains continue into
    /// ([`fillet_edges_tangent_chains`]).
    pub follow_tangent_chains: bool,
    /// Refuse (InvalidInput) a selection whose blends overlap on a face lying
    /// between two parallel picked edges with parallel walls beyond, unless
    /// the radius is half the walls' distance (the full round). By default
    /// the kernel builds such a pair as two balls each tangent to its own
    /// faces, pinching to a ridge where they meet; a caller that wants every
    /// ball to touch both faces it rolls on asks for the refusal
    /// (`brep_blend::fillet::pinch_refusal`).
    pub refuse_pinch: bool,
}

/// [`fillet_edges`] read with `options`: the selection widened along its
/// tangent chains first when asked, and the pinch refused when asked (judged
/// on the widened selection).
pub fn fillet_edges_with_options(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    options: FilletOptions,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut points = edge_points.to_vec();
    let mut names = edge_names.map(|names| names.to_vec());
    if options.follow_tangent_chains {
        let seeds = edge_points
            .iter()
            .map(|point| brep_blend::fillet::edges::resolve_edge_by_point(solid, *point))
            .collect::<Result<Vec<u64>, KernelRefusal>>()?;
        let added = crate::feature_pipeline::features::common::tangent_chain_expansion(solid, &seeds, &seeds)
            .map_err(|error| KernelRefusal::internal(KernelStage::Collect, "tangent_chain", error))?;
        points.extend(added.iter().map(|(_, point, _)| *point));
        if let Some(names) = names.as_mut() {
            names.extend(added.iter().map(|(_, _, name)| name.clone()));
        }
    }
    if options.refuse_pinch {
        if let Some(refusal) = brep_blend::fillet::pinch_refusal(solid, &points, radius) {
            return Err(refusal);
        }
    }
    let refused = match fillet_edges(solid, &points, names.as_deref(), radius, false, name) {
        Ok(built) => return Ok(built),
        Err(refused) => refused,
    };
    // A sliver the round swallows: answered where the plain build refuses.
    match swallowed_face_route(solid, &points, names.as_deref(), radius, name) {
        Some(Ok(built)) => Ok(built),
        _ => Err(refused),
    }
}

fn is_plane(face: &crate::topology::FaceRecord) -> bool {
    let s = &face.surface;
    s.degree_u == 1 && s.degree_v == 1 && s.control_points.len() == 2 && s.control_points[0].len() == 2 && face.surface.analytic().is_some_and(|a| matches!(a, crate::AnalyticSurface::Plane { .. }))
}

/// A SWALLOWED FACE: two parallel straight picked edges bound a planar strip
/// between two planar faces that meet at a convex angle beyond it (a corner
/// cut back by a sliver), and the round of that corner (its contacts
/// `r · cot(φ/2)` from the corner along each face, φ the angle inside the
/// material) reaches past both of the strip's edges: a ball of the radius
/// cannot touch the strip, it rolls on the two faces beyond, and the strip
/// goes. Every such strip is deleted in one heal (its neighbours extended to
/// their meeting edge) and those edges filleted with the rest of the
/// selection, each corner taking the first of its pair's names. `None`
/// where no such pair is picked, or the heal refuses.
fn swallowed_face_route(solid: &BrepSolid, edge_points: &[Vec3], edge_names: Option<&[String]>, radius: f64, name: Option<&str>) -> Option<Result<BrepSolid, KernelRefusal>> {
    let ids: Vec<u64> = edge_points.iter().map(|p| brep_blend::fillet::edges::resolve_edge_by_point(solid, *p).ok()).collect::<Option<_>>()?;
    let mut taken = vec![false; ids.len()];
    let mut strips: Vec<u64> = Vec::new();
    let mut corners: Vec<(usize, Vec3)> = Vec::new(); // (the pair's first index, a point on its corner edge)
    for i in 0..ids.len() {
        for j in i + 1..ids.len() {
            if taken[i] || taken[j] {
                continue;
            }
            let Ok((ea, fa)) = brep_blend::fillet::analyze::edge_and_faces(solid, ids[i]) else { continue };
            let Ok((eb, fb)) = brep_blend::fillet::analyze::edge_and_faces(solid, ids[j]) else { continue };
            if ea.curve.degree != 1 || eb.curve.degree != 1 || fa.len() != 2 || fb.len() != 2 {
                continue;
            }
            let ends = |e: &crate::topology::EdgeRecord| -> Option<(Vec3, Vec3)> { Some((e.curve.evaluate(e.t0).ok()?, e.curve.evaluate(e.t1).ok()?)) };
            let (Some((a0, a1)), Some((b0, b1))) = (ends(ea), ends(eb)) else { continue };
            let (Ok(da), Ok(db)) = (a1.sub(a0).normalized(), b1.sub(b0).normalized()) else { continue };
            if da.cross(db).length() > 1e-9 {
                continue;
            }
            // The strip: the face both edges bound, its one loop of four.
            let Some(strip) = fa.iter().find(|f| fb.iter().any(|g| g.id == f.id)) else { continue };
            if !is_plane(strip) || strip.loops.len() != 1 || strip.loops[0].coedges.len() != 4 || strips.contains(&strip.id) {
                continue;
            }
            let (pa, pb) = (a0.add(a1).scale(0.5), b0.add(b1).scale(0.5));
            // The faces beyond, planar, meeting at a convex angle (not parallel).
            let (Some(beyond_a), Some(beyond_b)) = (fa.iter().find(|f| f.id != strip.id), fb.iter().find(|f| f.id != strip.id)) else { continue };
            if !is_plane(beyond_a) || !is_plane(beyond_b) {
                continue;
            }
            let (Ok((on_a, na)), Ok((on_b, nb))) = (brep_blend::fillet::analyze::face_plane_normal(beyond_a, pa), brep_blend::fillet::analyze::face_plane_normal(beyond_b, pb)) else { continue };
            if na.cross(nb).length() < 1e-9 {
                continue;
            }
            // Their meeting line, at the strip's middle along the edges.
            let middle = pa.add(pb).scale(0.5);
            let det = na.dot(nb.cross(da));
            if det.abs() < 1e-12 {
                continue;
            }
            let corner = nb.cross(da).scale(na.dot(on_a)).add(da.cross(na).scale(nb.dot(on_b))).add(na.cross(nb).scale(da.dot(middle))).scale(1.0 / det);
            // Convex: the strip's edges lie inside the material's wedge,
            // behind both outward normals from the corner.
            if !(pa.sub(corner).dot(nb) < 0.0 && pb.sub(corner).dot(na) < 0.0) {
                continue;
            }
            // Swallowed: the corner's round reaches past both strip edges.
            let inside = std::f64::consts::PI - na.dot(nb).clamp(-1.0, 1.0).acos();
            if !(inside > 1e-9 && inside < std::f64::consts::PI - 1e-9) {
                continue;
            }
            let setback = radius / (0.5 * inside).tan();
            let from_corner = |p: Vec3| {
                let off = p.sub(corner);
                off.sub(da.scale(off.dot(da))).length()
            };
            if !(setback >= from_corner(pa) && setback >= from_corner(pb)) {
                continue;
            }
            taken[i] = true;
            taken[j] = true;
            strips.push(strip.id);
            corners.push((i, corner));
        }
    }
    if strips.is_empty() {
        return None;
    }
    let healed = crate::delete_faces_and_heal(solid, &strips).ok()?;
    let mut points: Vec<Vec3> = Vec::new();
    let mut names: Vec<String> = Vec::new();
    for (k, point) in edge_points.iter().enumerate() {
        if !taken[k] {
            points.push(*point);
            if let Some(given) = edge_names {
                names.push(given.get(k).cloned().unwrap_or_default());
            }
        }
    }
    for &(first, corner) in &corners {
        points.push(corner);
        if let Some(given) = edge_names {
            names.push(given.get(first).cloned().unwrap_or_default());
        }
    }
    Some(fillet_edges(&healed, &points, edge_names.map(|_| names.as_slice()), radius, false, name))
}

