//! A shell whose wall is joined by arcs: every point of the wall is within the
//! thickness of a kept face.
//!
//! [`crate::offset_shell`] joins the moved faces sharp, extending them until
//! they meet. Here the wall is the parallel one: inward, the solid less
//! everything at least the thickness from its kept faces, so the cavity is
//! rounded along the solid's concave edges; outward, everything outside the
//! solid within the thickness of its kept faces, so the wall is rounded about
//! the solid's convex edges and corners. Both come from the whole-solid
//! parallel offset ([`crate::offset_solid`] with
//! [`crate::OffsetCorners::Rounded`]), which is exact; an opening is a flat
//! face, whose plane ends the wall.
use crate::topology::FaceRecord;
use crate::{
    boolean_operation, boolean_operation_nary, offset_solid, split_solid_by_plane, thicken_trimmed_sheet, BooleanOperation, BooleanOptions, BrepSolid,
    KernelRefusal, KernelStage, NurbsCurve, OffsetCorners, OrRefuse, Vec3,
};

/// The prefix of every refusal of this operation.
pub const SHELL_ROUNDED_REFUSAL: &str = "offsetShellRounded";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{SHELL_ROUNDED_REFUSAL}: {message}"))
}

/// A point of a flat face's plane and its outward unit normal; refuses a face
/// that is not flat.
fn opening_plane(face: &FaceRecord) -> Result<(Vec3, Vec3), KernelRefusal> {
    let stage = KernelStage::Collect;
    let surface = &face.surface;
    let [u0, u1] = surface.domain_u().or_refuse(stage, "offsetShellRounded.domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(stage, "offsetShellRounded.domain")?;
    let at = |a: f64, b: f64| (u0 + (u1 - u0) * a, v0 + (v1 - v0) * b);
    let (u, v) = at(0.5, 0.5);
    let origin = surface.evaluate(u, v).or_refuse(stage, "offsetShellRounded.point")?;
    let normal = surface.normal(u, v).or_refuse(stage, "offsetShellRounded.normal")?;
    let size = surface.evaluate(u0, v0).or_refuse(stage, "offsetShellRounded.point")?.sub(surface.evaluate(u1, v1).or_refuse(stage, "offsetShellRounded.point")?).length();
    for a in [0.0, 0.25, 0.5, 0.75, 1.0] {
        for b in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let (u, v) = at(a, b);
            let point = surface.evaluate(u, v).or_refuse(stage, "offsetShellRounded.point")?;
            if point.sub(origin).dot(normal).abs() > 1e-9 * size {
                return Err(KernelRefusal::unsupported(stage, "opening_flat", format!("{SHELL_ROUNDED_REFUSAL}: opening face {} is not flat", face.id)));
            }
        }
    }
    Ok((origin, if face.same_sense { normal } else { normal.scale(-1.0) }))
}

/// `source` shelled to a wall `distance` thick, joined by arcs, open at the
/// flat faces `opening_face_ids`. The sign is [`crate::offset_shell`]'s: a
/// positive distance puts the wall inside the solid, a negative one outside.
///
/// Inward, the wall is the solid less every point of it at least the distance
/// from its kept faces. Outward, it is every point outside the solid within
/// the distance of its kept faces, ended by the openings' planes. With no
/// opening the result is the solid's own shell and the parallel one, inside or
/// outside it.
///
/// Refuses an opening that is not flat, an inward wall with two openings that
/// share an edge, and whatever the parallel offset or
/// the Booleans refuse.
pub fn offset_shell_rounded(source: &BrepSolid, opening_face_ids: &[u64], distance: f64) -> Result<BrepSolid, KernelRefusal> {
    if !distance.is_finite() || distance.abs() <= 1e-12 {
        return Err(input("distance", "distance must be a nonzero finite value".into()));
    }
    let d = distance.abs();
    let mut openings = Vec::with_capacity(opening_face_ids.len());
    for id in opening_face_ids {
        let face = source.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == *id).ok_or_else(|| input("opening", format!("the solid has no face {id}")))?;
        openings.push((face, opening_plane(face)?));
    }
    let options = BooleanOptions::default();
    if distance > 0.0 {
        // two openings that share an edge would need the corner between their run-ons filled too
        let edges = |face: &FaceRecord| -> Vec<u64> { face.loops.iter().flat_map(|record| &record.coedges).map(|coedge| coedge.edge_id).collect() };
        for (index, (first, _)) in openings.iter().enumerate() {
            for (second, _) in openings.iter().skip(index + 1) {
                if edges(first).iter().any(|edge| edges(second).contains(edge)) {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Collect,
                        "openings_adjacent",
                        format!("{SHELL_ROUNDED_REFUSAL}: openings {} and {} share an edge; an inward wall open at two faces that meet is not built", first.id, second.id),
                    ));
                }
            }
        }
        // the solid run on past each opening, so that the cavity comes out through it
        let mut run_on = vec![source.clone()];
        for (face, _) in &openings {
            let loops: Vec<Vec<NurbsCurve>> = face.loops.iter().map(|record| record.coedges.iter().map(|coedge| coedge.pcurve.clone()).collect()).collect();
            let outward = if face.same_sense { 2.0 * d } else { -2.0 * d };
            run_on.extend(thicken_trimmed_sheet(&face.surface, &loops, outward, false)?);
        }
        let body = if run_on.len() == 1 { run_on.remove(0) } else { boolean_operation_nary(&run_on, BooleanOperation::Union)? };
        let cavity = offset_solid(&body, -d, OffsetCorners::Rounded)?.solid;
        boolean_operation(source, &cavity, BooleanOperation::Subtract, &options)
    } else {
        let mut grown = offset_solid(source, d, OffsetCorners::Rounded)?.solid;
        for (_, (origin, normal)) in &openings {
            grown = split_solid_by_plane(&grown, *origin, *normal).or_refuse(KernelStage::Fragment, "offsetShellRounded.split")?.0;
        }
        boolean_operation(&grown, source, BooleanOperation::Subtract, &options)
    }
}

