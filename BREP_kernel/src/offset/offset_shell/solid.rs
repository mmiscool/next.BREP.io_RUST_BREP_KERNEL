//! Whole-solid offset (D3).
//!
//! `offset_solid` runs the shell pipeline in its WHOLE-BODY mode
//! (`Body::Whole`): every face is retained, there is no opening and no wall,
//! the source's own faces are not assembled, and the body is the offset
//! carriers alone, closed at every sharp junction by the extend-and-intersect
//! the pipeline already does for a shell's retained faces.
//!
//! The sign is the natural one: a positive distance grows the body along its
//! outward normals, a negative one shrinks it. `offset_shell` takes the
//! opposite sign (positive = inward wall), so the pipeline is asked for
//! `-distance`.
//!
//! `OffsetCorners::Rounded` (D3) is the true parallel body, the Minkowski sum
//! with a ball of radius `|d|`: the sharp body with every convex edge image
//! (outward) or concave edge image (inward) rolled by the blend network at
//! radius `|d|`, which is what puts the exact cylinder or torus cap on each
//! straight or circular edge and the corner ball at each vertex. The blend's
//! constructions are reused through `fillet_edges`, never rebuilt here.
use super::pipeline::{offset_body, Body};
use super::{OffsetFaceRole, OffsetShellFaceImageRecord, OffsetShellResultRecord};
use crate::classification::PolygonClass;
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{
    boolean_operation, fillet_edges, project_point_to_surface, BooleanOperation, BooleanOptions,
    KernelRefusal, KernelStage, OrRefuse, RefusalClass, Vec2, Vec3,
};
use rustc_hash::FxHashMap as HashMap;
use serde::{Deserialize, Serialize};

/// How the offset body closes the joins between offset faces.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum OffsetCorners {
    #[default]
    Sharp,
    /// The exact parallel body: convex joins (outward) or concave joins
    /// (inward) carry the rolling ball's cylinder, torus or sphere.
    Rounded,
}

/// Offset every face of `source` by `distance` (positive grows) and join the
/// offset faces into a closed body.
pub fn offset_solid(
    source: &BrepSolid,
    distance: f64,
    corners: OffsetCorners,
) -> Result<OffsetShellResultRecord, KernelRefusal> {
    let sharp = offset_body(source, &[], -distance, Body::Whole, "offset_solid")?;
    match corners {
        OffsetCorners::Sharp => Ok(sharp),
        OffsetCorners::Rounded => round_offset_joins(sharp, distance),
    }
}

/// Roll the sharp body's joins that the ball of radius `|distance|` rounds:
/// the convex edges of an outward offset, the concave edges of an inward one.
/// A body with no such edge is already the parallel body and is returned as
/// it is. Provenance: a face still on a sharp-body surface keeps that face's
/// image; a cap (a cylinder, torus or corner ball the blend built, which lies
/// on no offset face) is an `Offset` image with `source_face_id` 0, because it
/// rounds a JOIN between source faces and has no single source face.
fn round_offset_joins(
    sharp: OffsetShellResultRecord,
    distance: f64,
) -> Result<OffsetShellResultRecord, KernelRefusal> {
    let wanted_convex = distance > 0.0;
    let midpoints = sharp_join_midpoints(&sharp.solid, wanted_convex)?;
    if midpoints.is_empty() {
        return Ok(sharp);
    }
    let solid = fillet_edges(&sharp.solid, &midpoints, None, distance.abs(), false, None)
        .map_err(|error| {
            error.with_message(|message| {
                format!(
                    "offset_solid: the rounded joins of the offset body could not be built \
                     ({} edge(s) at radius {:.6}): {message}",
                    midpoints.len(),
                    distance.abs()
                )
            })
        })?;
    let operands = sharp
        .solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .zip(&sharp.face_images)
        .map(|(face, image)| (face, image.clone()))
        .collect::<Vec<_>>();
    let cap = OffsetShellFaceImageRecord {
        role: OffsetFaceRole::Offset,
        source_face_id: 0,
    };
    let face_images = provenance_by_surface(&solid, &operands, Some(&cap))?;
    Ok(OffsetShellResultRecord { solid, face_images })
}

/// Per-face provenance of a body whose faces lie on its operands' surfaces (a
/// boolean result, a blend's untouched faces): each result face takes the
/// image of the FIRST operand face whose surface carries an interior point of
/// it and whose trim contains that point's foot. Operands are matched in the
/// order given, so a coincident pair (a held face and the opening it copies)
/// resolves to whichever is listed first. A face on no operand surface takes
/// `unmatched`, or refuses by name when there is none.
fn provenance_by_surface(
    result: &BrepSolid,
    operands: &[(&FaceRecord, OffsetShellFaceImageRecord)],
    unmatched: Option<&OffsetShellFaceImageRecord>,
) -> Result<Vec<OffsetShellFaceImageRecord>, KernelRefusal> {
    let band = 1e-6 * crate::solid_scale(result).max(1.0);
    let mut images = Vec::new();
    for face in result.shells.iter().flat_map(|shell| &shell.faces) {
        let seeds = super::source_seeds(result, face.id)?;
        let seed = seeds[0];
        let point = face.surface.evaluate(seed.x, seed.y).or_refuse(KernelStage::Sew, "evaluate")?;
        let mut found = None;
        for (operand, image) in operands {
            let Ok(foot) = project_point_to_surface(&operand.surface, point) else { continue };
            if foot.distance > band {
                continue;
            }
            let inside = super::parameter_point_in_face(operand, Vec2 { x: foot.u, y: foot.v }, 1e-8)
                .is_ok_and(|class| class != PolygonClass::Outside);
            if inside {
                found = Some(image.clone());
                break;
            }
        }
        match found.or_else(|| unmatched.cloned()) {
            Some(image) => images.push(image),
            None => {
                return Err(KernelRefusal::internal(
                    KernelStage::Sew,
                    "offset_shell_provenance_lost",
                    format!(
                        "offset_shell: result face {} at ({:.4}, {:.4}, {:.4}) lies on no operand face",
                        face.id, point.x, point.y, point.z
                    ),
                ))
            }
        }
    }
    Ok(images)
}

/// Midpoints of the SHARP edges of `solid` whose convexity is `convex`: an
/// edge bordering two different faces whose outward normals differ there, read
/// at the edge midpoint. Tangent junctions (a blend face meeting its support)
/// and face-local seams are not joins and are skipped.
fn sharp_join_midpoints(solid: &BrepSolid, convex: bool) -> Result<Vec<Vec3>, KernelRefusal> {
    // Cosine of the dihedral angle above which two faces read as tangent.
    const TANGENT_COSINE: f64 = 1.0 - 1e-6;
    let faces = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    let mut uses = HashMap::<u64, Vec<(&FaceRecord, bool)>>::default();
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            uses.entry(coedge.edge_id).or_default().push((face, coedge.forward));
        }
    }
    let mut midpoints = Vec::new();
    for edge in &solid.edges {
        if edge.degenerate {
            continue;
        }
        let Some(pair) = uses.get(&edge.id) else { continue };
        if pair.len() != 2 || pair[0].0.id == pair[1].0.id {
            continue;
        }
        if edge_is_convex_join(edge, pair[0], pair[1].0, TANGENT_COSINE)? == Some(convex) {
            midpoints.push(edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).or_refuse(KernelStage::Classify, "evaluate")?);
        }
    }
    Ok(midpoints)
}

/// `Some(true)` for a convex join, `Some(false)` for a concave one, `None` for
/// a tangent junction. With `t` the direction the first face's coedge runs
/// along the edge and `n_a` that face's outward normal, `n_a × t` points into
/// the first face; the join is convex when the second face's outward normal
/// points away from there.
fn edge_is_convex_join(
    edge: &EdgeRecord,
    (first, first_forward): (&FaceRecord, bool),
    second: &FaceRecord,
    tangent_cosine: f64,
) -> Result<Option<bool>, KernelRefusal> {
    let t = 0.5 * (edge.t0 + edge.t1);
    let derivatives = edge.curve.derivatives(t, 1).or_refuse(KernelStage::Classify, "derivatives")?;
    let point = derivatives[0];
    let mut tangent = derivatives[1].normalized().or_refuse(KernelStage::Classify, "normalized")?;
    if !first_forward {
        tangent = tangent.scale(-1.0);
    }
    let normal_a = outward_normal_at(first, point)?;
    let normal_b = outward_normal_at(second, point)?;
    if normal_a.dot(normal_b) >= tangent_cosine {
        return Ok(None);
    }
    Ok(Some(normal_b.dot(normal_a.cross(tangent)) < 0.0))
}

fn outward_normal_at(face: &FaceRecord, point: Vec3) -> Result<Vec3, KernelRefusal> {
    let foot = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Classify, "project_point_to_surface")?;
    super::face_normal(face, foot.u, foot.v)
}

// ---------------------------------------------------------------------------
// Shell as a boolean
// ---------------------------------------------------------------------------

/// The textbook shell: the offset body with the opening faces HELD (copied at
/// distance zero, so they cancel in the difference), then `source − body` for
/// an inward shell and `body − source` for an outward one. The walls at the
/// openings come out of the boolean. `distance` carries `offset_shell`'s sign
/// (positive inward).
///
/// This is the lane `offset_shell` falls back to when its direct construction
/// refuses by one of its own names ([`direct_refusal_takes_the_boolean_lane`]);
/// it is public so the two can be compared on any fixture. The result's
/// `face_images` are read back from the operands by surface
/// ([`provenance_by_surface`]): `Offset` for a face of the body's skins,
/// `Wall` for a held opening face, `Source` for a face of the source.
pub fn offset_shell_by_boolean(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
) -> Result<OffsetShellResultRecord, KernelRefusal> {
    if opening_face_ids.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "offset_shell_no_opening",
            "offset_shell: at least one opening face is required",
        ));
    }
    let body = offset_body(source, opening_face_ids, distance, Body::Whole, "offset_shell_by_boolean")?;
    let (kept, removed) = if distance > 0.0 { (source, &body.solid) } else { (&body.solid, source) };
    let solid = boolean_operation(kept, removed, BooleanOperation::Subtract, &BooleanOptions::default())
        .map_err(|error| {
            error.with_message(|message| {
                format!("offset_shell: the boolean lane's difference could not be built: {message}")
            })
        })?;
    // The body's faces first, so the ring on a held face's plane reads as
    // that face's `Wall` rather than as the coincident source opening.
    let mut operands = body
        .solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .zip(&body.face_images)
        .map(|(face, image)| (face, image.clone()))
        .collect::<Vec<_>>();
    operands.extend(source.shells.iter().flat_map(|shell| &shell.faces).map(|face| {
        (
            face,
            OffsetShellFaceImageRecord {
                role: OffsetFaceRole::Source,
                source_face_id: face.id,
            },
        )
    }));
    let face_images = provenance_by_surface(&solid, &operands, None)?;
    Ok(OffsetShellResultRecord { solid, face_images })
}

/// Whether a refusal of the direct pipeline is one of its OWN named
/// inabilities, the ones the boolean recipe can answer differently: an
/// `offset_shell_*` unsupported-geometry or internal refusal, or the
/// assembler's typed non-integral-genus and degenerate-arrangement refusals
/// that `cause_in_front` carries a shell cause through. Input refusals
/// (distance, foreign opening, no retained face) and soundness refusals are
/// not: the first name a caller's mistake, the second a body that must not be
/// shipped by any route.
pub(super) fn direct_refusal_takes_the_boolean_lane(refusal: &KernelRefusal) -> bool {
    match &refusal.class {
        RefusalClass::UnsupportedGeometry { what } | RefusalClass::Internal { what } => {
            what.starts_with("offset_shell_")
        }
        RefusalClass::NonIntegralGenus { .. } | RefusalClass::DegenerateArrangement { .. } => {
            refusal.message.starts_with("offset_shell")
        }
        _ => false,
    }
}
