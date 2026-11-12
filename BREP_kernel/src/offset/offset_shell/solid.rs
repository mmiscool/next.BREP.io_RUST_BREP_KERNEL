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
use super::debug_enabled;
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
    let grown = offset_solid_joined(source, distance, corners)?;
    if corners == OffsetCorners::Rounded && distance > 0.0 {
        if let Some(carved) = super::bore_dimples::carve_bore_dimples(source, &grown.solid, distance) {
            let operands = grown
                .solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .zip(&grown.face_images)
                .map(|(face, image)| (face, image.clone()))
                .collect::<Vec<_>>();
            let cap = OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 };
            if let Ok(face_images) = provenance_by_surface(&carved, &operands, Some(&cap)) {
                return Ok(OffsetShellResultRecord { solid: carved, face_images });
            }
        }
    }
    Ok(grown)
}

/// [`offset_solid`] before the bore dimples ([`super::bore_dimples`]): the
/// sharp body, or its rounding, or a decomposition's.
fn offset_solid_joined(
    source: &BrepSolid,
    distance: f64,
    corners: OffsetCorners,
) -> Result<OffsetShellResultRecord, KernelRefusal> {
    if corners == OffsetCorners::Rounded && distance > 0.0 {
        if let Some(grown) = grown_across_reflex_edges(source, distance) {
            return Ok(grown);
        }
    }
    let sharp = match offset_body(source, &[], -distance, Body::Whole, "offset_solid") {
        Ok(sharp) => sharp,
        Err(error) => {
            if corners == OffsetCorners::Rounded {
                if let Some(grown) = grown_by_pieces(source, distance) {
                    return Ok(grown);
                }
            }
            return Err(error);
        }
    };
    match corners {
        OffsetCorners::Sharp => Ok(sharp),
        OffsetCorners::Rounded => match round_offset_joins(sharp, distance) {
            Ok(rounded) => Ok(rounded),
            // A mitred body cannot carry the ball over a vertex of degree four
            // or more; a convex polyhedron's parallel body is built directly.
            Err(error) => match crate::blend::convex_parallel::convex_polyhedron_parallel(source, distance) {
                Ok(Some((solid, images))) if solid.validate().is_empty() => {
                    let face_images = images
                        .into_iter()
                        .map(|image| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: image.unwrap_or(0) })
                        .collect();
                    Ok(OffsetShellResultRecord { solid, face_images })
                }
                _ => Err(error),
            },
        },
    }
}

/// `BREP_OFFSET_GROW_BY_PIECES` (default on; `=0` restores): [`grown_by_pieces`].
fn grow_by_pieces_enabled() -> bool {
    switch!("BREP_OFFSET_GROW_BY_PIECES", true)
}


/// The ROUNDED growth of a body whose own offset body the pipeline could not
/// assemble, as the union of its pieces' growths: the parallel body of a
/// union is the union of the parallel bodies, `(A ∪ B) ⊕ D = (A ⊕ D) ∪
/// (B ⊕ D)`, for any pieces whose union is the body.
///
/// Cause (a drum with a groove 1.6 wide grown by 1, brep-bench
/// offset/grooved-drum-grown): the groove's walls offset across each other,
/// "a skin partly shadowed that way is not yet assembled", and even assembled
/// the sharp body would have no groove left to round, while the case's notch
/// is the two tori about the groove's outer rims. Cut at the planes of its
/// planar faces the drum is three cylinders; each grows by the pipeline's own
/// rounded offset, and their union carries the notch where the two rim tori
/// cross.
///
/// The cuts are chosen by [`thin_pair_planes`] (the planes parallel to a pair
/// of planar faces whose offsets cross), each kept only where it divides a
/// piece (`split_solid_by_plane`). A SHRINK runs the same construction on the
/// body's complement in a frame box ([`shrunk_by_pieces`]). Every piece must
/// grow and the union must validate; anything else declines and the
/// pipeline's own refusal stands. The faces are all `Offset` images with
/// `source_face_id` 0, as the rounded caps are: a face of the union may carry
/// the growth of several source faces' pieces.
fn grown_by_pieces(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !grow_by_pieces_enabled() {
        return None;
    }
    match grown_by_pieces_impl(source, distance) {
        Ok(result) => result,
        Err(error) => {
            os_debug!("grow by pieces declined: {error}");
            None
        }
    }
}

fn grown_by_pieces_impl(source: &BrepSolid, distance: f64) -> Result<Option<OffsetShellResultRecord>, KernelRefusal> {
    if distance < 0.0 {
        return shrunk_by_pieces(source, -distance);
    }
    let Some(sets) = thin_pair_planes(source, distance, true)? else {
        return Ok(None);
    };
    let mut answer = None;
    for planes in &sets {
        match grown_union(source, planes, distance) {
            Ok(Some(solid)) => {
                answer = Some(solid);
                break;
            }
            Ok(None) => {}
            Err(error) => os_debug!("grow by pieces: {} plane(s) declined: {error}", planes.len()),
        }
    }
    let Some(solid) = answer else {
        return Ok(None);
    };
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
        .collect();
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
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
/// refuses by one of its own names (`direct_refusal_takes_the_boolean_lane`);
/// it is public so the two can be compared on any fixture. The result's
/// `face_images` are read back from the operands by surface
/// (`provenance_by_surface`): `Offset` for a face of the body's skins,
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
    let body = offset_body(source, opening_face_ids, distance, Body::Whole, "offset_shell_by_boolean")
        .or_else(|error| {
            if !direct_refusal_takes_the_boolean_lane(&error) {
                return Err(error);
            }
            let result = super::joint_offset::build(source, opening_face_ids, distance);
            if let Err(reason) = &result {
                os_debug!("joint offset refused: {reason}");
            }
            result.map_err(|_| error)
        })?;
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

/// Lane O row 34: the CLOSED hollow, a shell with no openings. The offset
/// body, the whole source grown or shrunk with sharp joins, against the
/// source: `source − body` inward and `body − source` outward, so the result
/// is the outer shell and the inner void's shell, with the walls of the band
/// between them. `distance` carries `offset_shell`'s sign (positive inward).
/// `face_images` are read back from the operands by surface as in
/// [`offset_shell_by_boolean`]: `Offset` for the body's faces, `Source` for
/// the source's.
///
/// The construction checks itself: the result is exactly two shells and its
/// volume is the difference of the operands' volumes.
pub fn offset_shell_closed(source: &BrepSolid, distance: f64) -> Result<OffsetShellResultRecord, KernelRefusal> {
    let body = offset_body(source, &[], distance, Body::Whole, "offset_shell_closed")?;
    let (kept, removed) = if distance > 0.0 { (source, &body.solid) } else { (&body.solid, source) };
    let solid = boolean_operation(kept, removed, BooleanOperation::Subtract, &BooleanOptions::default())
        .map_err(|error| {
            error.with_message(|message| format!("offset_shell: the closed hollow's difference could not be built: {message}"))
        })?;
    let volume = |solid: &BrepSolid| {
        crate::solid_mass_properties(solid)
            .map(|properties| properties.volume)
            .or_refuse(KernelStage::Validate, "solid_mass_properties")
    };
    let expected = volume(kept)? - volume(removed)?;
    let measured = volume(&solid)?;
    if solid.shells.len() != 2 || !(expected > 0.0) || !((measured - expected).abs() <= 1e-9 * expected) {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "offset_shell_closed_hollow",
            format!(
                "offset_shell: the closed hollow is not the band between its operands ({} shells, volume {measured} against {expected})",
                solid.shells.len()
            ),
        ));
    }
    let mut operands = body
        .solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .zip(&body.face_images)
        .map(|(face, image)| (face, image.clone()))
        .collect::<Vec<_>>();
    operands.extend(source.shells.iter().flat_map(|shell| &shell.faces).map(|face| {
        (face, OffsetShellFaceImageRecord { role: OffsetFaceRole::Source, source_face_id: face.id })
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
        RefusalClass::NonIntegralGenus { .. } => {
            refusal.message.starts_with("offset_shell")
                || (bare_genus_takes_the_boolean_lane() && refusal.message.starts_with("boolean assembly:"))
        }
        RefusalClass::DegenerateArrangement { .. } => refusal.message.starts_with("offset_shell"),
        _ => false,
    }
}

/// `BREP_OFFSET_BARE_GENUS_BOOLEAN_LANE` (default on; `=0` restores): the
/// direct pipeline's assembly refusing a non-integral genus takes the boolean
/// lane even with no shell cause put in front of it. Which sentence a refusal
/// carries says nothing about whether the lane can answer:
/// `inbox_20261017_negative_shell` only reached the lane through the note of
/// an omission, and the omission was a misread fillet's true skin; reading
/// that skin correctly (`BREP_OFFSET_TANGENT_EDGE_EUCLID`) left the same
/// failed assembly with no note, and the shell was refused.
fn bare_genus_takes_the_boolean_lane() -> bool {
    crate::switch!("BREP_OFFSET_BARE_GENUS_BOOLEAN_LANE", true)
}

/// The cut planes: every planar face's plane parallel to a THIN PAIR, two
/// planar faces with opposite outward normals closer than `2·amount` that face
/// each other across void (`grow`) or across material (shrink) — the pair
/// whose offsets cross. Deduplicated. Two sets are offered in turn: every
/// planar face's plane parallel to a thin pair, then the thin pairs' own
/// planes alone (when fewer), because the boolean that unions the grown
/// pieces can fail on where one set's pieces meet and not the other's (the
/// dumbbell's complement cut at all four y planes left sliver edges where the
/// neck-mouth rounds of adjacent pieces cross). `None` when there is no thin
/// pair or every set has more than eight planes.
fn thin_pair_planes(solid: &BrepSolid, amount: f64, grow: bool) -> Result<Option<Vec<Vec<(Vec3, Vec3)>>>, KernelRefusal> {
    let band = 1e-9 * crate::solid_scale(solid).max(1.0);
    // Each planar face's plane, its OUTWARD normal.
    let mut faces: Vec<(Vec3, Vec3)> = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let Some((point, normal)) = super::rim_welds::planar_surface_frame(&face.surface, band)? else {
            continue;
        };
        let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let Ok(outward) = super::face_normal(face, 0.5 * (u0 + u1), 0.5 * (v0 + v1)) else {
            continue;
        };
        faces.push((point, if outward.dot(normal) < 0.0 { normal.scale(-1.0) } else { normal }));
    }
    let mut directions: Vec<Vec3> = Vec::new();
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    for (index, (point, normal)) in faces.iter().enumerate() {
        for (offset, (other, other_normal)) in faces[index + 1..].iter().enumerate() {
            if normal.dot(*other_normal) > -1.0 + 1e-12 {
                continue;
            }
            // Signed gap from this face's plane to the other's, along this
            // face's outward normal: positive across void, negative across
            // material.
            let gap = other.sub(*point).dot(*normal);
            let thin = if grow { gap > band && gap < 2.0 * amount } else { gap < -band && -gap < 2.0 * amount };
            if thin {
                pairs.push((index, index + 1 + offset));
                if !directions.iter().any(|axis| axis.dot(*normal).abs() >= 1.0 - 1e-12) {
                    directions.push(*normal);
                }
            }
        }
    }
    if directions.is_empty() {
        return Ok(None);
    }
    let push = |planes: &mut Vec<(Vec3, Vec3)>, point: Vec3, normal: Vec3| {
        let seen = planes
            .iter()
            .any(|(other, axis)| axis.dot(normal).abs() >= 1.0 - 1e-12 && point.sub(*other).dot(*axis).abs() <= band);
        if !seen {
            planes.push((point, normal));
        }
    };
    let mut parallel: Vec<(Vec3, Vec3)> = Vec::new();
    for (point, normal) in &faces {
        if directions.iter().any(|axis| axis.dot(*normal).abs() >= 1.0 - 1e-12) {
            push(&mut parallel, *point, *normal);
        }
    }
    let mut paired: Vec<(Vec3, Vec3)> = Vec::new();
    for (first, second) in &pairs {
        push(&mut paired, faces[*first].0, faces[*first].1);
        push(&mut paired, faces[*second].0, faces[*second].1);
    }
    let mut sets = Vec::new();
    if parallel.len() <= 8 {
        sets.push(parallel.clone());
    }
    if paired.len() <= 8 && paired.len() < parallel.len() {
        sets.push(paired);
    }
    Ok((!sets.is_empty()).then_some(sets))
}

/// `solid` cut by every plane that divides a piece of it.
fn cut_by_planes(solid: &BrepSolid, planes: &[(Vec3, Vec3)]) -> Vec<BrepSolid> {
    let mut pieces = vec![solid.clone()];
    for (point, normal) in planes {
        let mut next = Vec::with_capacity(pieces.len() * 2);
        for piece in pieces {
            match crate::split_solid_by_plane(&piece, *point, *normal) {
                Ok((below, above)) => {
                    next.push(below);
                    next.push(above);
                }
                Err(_) => next.push(piece),
            }
        }
        pieces = next;
    }
    pieces
}

/// The SHRINK by `amount` through the complement: `S ⊖ D = S − ((F − S) ⊕ D)`
/// for a frame box `F` reaching past `S` by more than `amount`, where the
/// complement's growth is the union of its pieces' growths, the pieces cut at
/// the planes of `S`'s planar faces.
///
/// Cause (a drum with a flange 1.6 thick shrunk by 1, brep-bench
/// offset/flanged-drum-shrunk): the flange's two faces offset across each
/// other ("a skin partly shadowed that way is not yet assembled"), and the
/// low ridge the case reads is the two concave tori about the flange's inner
/// rims, which the complement's pieces carry as the rounds of their own
/// convex rims.
fn shrunk_by_pieces(source: &BrepSolid, amount: f64) -> Result<Option<OffsetShellResultRecord>, KernelRefusal> {
    let Some(sets) = thin_pair_planes(source, amount, false)? else {
        return Ok(None);
    };
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for face in source.shells.iter().flat_map(|shell| &shell.faces) {
        for control in face.surface.control_points.iter().flatten() {
            let Ok(point) = control.point() else { return Ok(None) };
            low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
            high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
        }
    }
    let margin = 3.0 * amount;
    let corner = low.sub(Vec3::new(margin, margin, margin));
    let size = high.sub(low).add(Vec3::new(2.0 * margin, 2.0 * margin, 2.0 * margin));
    let frame = crate::make_box_brep(corner, size.x, size.y, size.z).or_refuse(KernelStage::Refine, "make_box_brep")?;
    let options = BooleanOptions::default();
    let outside = boolean_operation(&frame, source, BooleanOperation::Subtract, &options)?;
    let mut answer = None;
    for planes in &sets {
        let attempt = grown_union(&outside, planes, amount).and_then(|reach| match reach {
            Some(reach) => boolean_operation(source, &reach, BooleanOperation::Subtract, &options).map(Some),
            None => Ok(None),
        });
        match attempt {
            Ok(Some(solid)) if solid.validate().is_empty() => {
                answer = Some(solid);
                break;
            }
            Ok(_) => {}
            Err(error) => os_debug!("shrink by pieces: {} plane(s) declined: {error}", planes.len()),
        }
    }
    if answer.is_none() && shrink_by_subtraction_enabled() {
        for planes in &sets {
            match subtracted_one_by_one(source, &outside, planes, amount) {
                Ok(Some(solid)) => {
                    answer = Some(solid);
                    break;
                }
                Ok(None) => {}
                Err(error) => os_debug!("shrink by subtraction: {} plane(s) declined: {error}", planes.len()),
            }
        }
    }
    let Some(solid) = answer else {
        return Ok(None);
    };
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
        .collect();
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
}

/// `solid` cut at `planes`, each piece grown by `amount` with rounded joins,
/// and the growths' union; `None` when the planes leave one piece or the
/// union does not validate.
fn grown_union(solid: &BrepSolid, planes: &[(Vec3, Vec3)], amount: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let Some(grown) = grown_pieces(solid, planes, amount)? else {
        return Ok(None);
    };
    let union = crate::boolean_operation_nary(&grown, BooleanOperation::Union)?;
    Ok(union.validate().is_empty().then_some(union))
}

/// `solid` cut at `planes`, each piece grown by `amount` with rounded joins;
/// `None` when the planes leave one piece.
fn grown_pieces(solid: &BrepSolid, planes: &[(Vec3, Vec3)], amount: f64) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    let pieces = cut_by_planes(solid, planes);
    if pieces.len() < 2 {
        return Ok(None);
    }
    os_debug!("by pieces: {} plane(s) cut the body into {} piece(s)", planes.len(), pieces.len());
    let mut grown = Vec::with_capacity(pieces.len());
    for (index, piece) in pieces.iter().enumerate() {
        let sharp = offset_body(piece, &[], -amount, Body::Whole, "offset_solid_piece")
            .map_err(|error| error.with_message(|message| format!("piece {index}: {message}")))?;
        grown.push(round_offset_joins(sharp, amount)?.solid);
    }
    Ok(Some(grown))
}

/// `BREP_OFFSET_SHRINK_BY_SUBTRACTION` (default on; `=0` restores): where the union of a
/// shrink's grown complement pieces cannot be assembled, the pieces are
/// subtracted from the body one at a time instead.
///
/// Cause (the dumbbell plate shrunk by 1.5, brep-bench
/// shell/dumbbell-neck-filled): the grown pieces' concave rounds about the
/// neck mouths cross at y = 5, and their n-ary union failed ("boolean assembly
/// produced invalid topology (S=3 genus=2)", one-use edges ~3e-5 long near
/// (8.882, 5, 1.5)) for every plane set. `S − (A ∪ B) = (S − A) − B`.
fn shrink_by_subtraction_enabled() -> bool {
    switch!("BREP_OFFSET_SHRINK_BY_SUBTRACTION", true)
}


fn subtracted_one_by_one(source: &BrepSolid, outside: &BrepSolid, planes: &[(Vec3, Vec3)], amount: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let Some(grown) = grown_pieces(outside, planes, amount)? else {
        return Ok(None);
    };
    let options = BooleanOptions::default();
    let mut body = source.clone();
    for (index, piece) in grown.iter().enumerate() {
        body = boolean_operation(&body, piece, BooleanOperation::Subtract, &options)
            .map_err(|error| error.with_message(|message| format!("subtracting grown piece {index}: {message}")))?;
    }
    Ok(body.validate().is_empty().then_some(body))
}

/// `BREP_OFFSET_REFLEX_BY_PIECES` (default on; `=0` restores): [`grown_across_reflex_edges`].
fn reflex_by_pieces_enabled() -> bool {
    switch!("BREP_OFFSET_REFLEX_BY_PIECES", true)
}


/// The ROUNDED growth of a body with CONCAVE (reflex) edges between planar
/// faces, as the union of its pieces' growths, the pieces cut at those
/// faces' planes; `None` (and the ordinary sharp body + rounds) where the body
/// has none, or a piece or the union fails.
///
/// Cause (an L-shaped plate grown by 1, brep-bench offset/l-block-grown, read
/// volume +0.041 and area −0.061 against the closed form): the sharp body is
/// right, but where its reflex edge meets the top and bottom, the rounds of
/// the two convex edges beside it must meet in a sharp concave crease, and
/// `fillet_edges` blends that corner instead. Cut at x = 5 the plate is two
/// boxes; each grows to a rounded box exactly, and their union carries the
/// crease as the two rounds' own intersection.
fn grown_across_reflex_edges(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !reflex_by_pieces_enabled() {
        return None;
    }
    let attempt = || -> Result<Option<OffsetShellResultRecord>, KernelRefusal> {
        let band = 1e-9 * crate::solid_scale(source).max(1.0);
        let faces = source.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
        let mut uses = HashMap::<u64, Vec<(&FaceRecord, bool)>>::default();
        for face in &faces {
            for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                uses.entry(coedge.edge_id).or_default().push((face, coedge.forward));
            }
        }
        let mut planes: Vec<(Vec3, Vec3)> = Vec::new();
        for edge in &source.edges {
            if edge.degenerate {
                continue;
            }
            let Some(pair) = uses.get(&edge.id) else { continue };
            if pair.len() != 2 || pair[0].0.id == pair[1].0.id {
                continue;
            }
            if edge_is_convex_join(edge, pair[0], pair[1].0, 1.0 - 1e-6)? != Some(false) {
                continue;
            }
            for (face, _) in pair {
                let Some((point, normal)) = super::rim_welds::planar_surface_frame(&face.surface, band)? else {
                    continue;
                };
                let seen = planes
                    .iter()
                    .any(|(other, axis)| axis.dot(normal).abs() >= 1.0 - 1e-12 && point.sub(*other).dot(*axis).abs() <= band);
                if !seen {
                    planes.push((point, normal));
                }
            }
        }
        if planes.is_empty() || planes.len() > 8 {
            return Ok(None);
        }
        // One plane per reflex edge is enough, and fewer pieces meet in fewer
        // places: try each plane alone first, then all of them.
        let mut sets = planes.iter().map(|plane| vec![*plane]).collect::<Vec<_>>();
        if planes.len() > 1 {
            sets.push(planes.clone());
        }
        for set in &sets {
            match grown_union(source, set, distance) {
                Ok(Some(solid)) => {
                    os_debug!("reflex by pieces: {} plane(s)", set.len());
                    let face_images = solid
                        .shells
                        .iter()
                        .flat_map(|shell| &shell.faces)
                        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
                        .collect();
                    return Ok(Some(OffsetShellResultRecord { solid, face_images }));
                }
                Ok(None) => {}
                Err(error) => os_debug!("reflex by pieces: {} plane(s) declined: {error}", set.len()),
            }
        }
        Ok(None)
    };
    match attempt() {
        Ok(result) => result,
        Err(error) => {
            os_debug!("reflex by pieces declined: {error}");
            None
        }
    }
}
