//! PF — Push Face.
//!
//! Pushes/pulls faces of a single solid along their outward normal field by a
//! signed `distance` (positive grows the solid, negative shrinks it). Every
//! canonical carrier kind has a guarded route; supported trim/neighbour slices
//! vary by carrier and unsupported combinations return a typed kernel error.
//!
//! AUDIT CORRECTION (task premise): the task states "there is NO dedicated
//! push_face kernel op — the caller builds a tool + boolean". That is falsified by
//! [`crate::move_faces`] (Golovanov §6.12 direct-edit push, exported at
//! `lib.rs:243`): it translates a face record in place and re-intersects the
//! neighbours. This port uses it for the planar case for a decisive reason —
//! NAME FIDELITY (contract rule 2). The retired extrude-tool + boolean path makes the
//! pushed face and the tool cap COINCIDENT interior faces that the union deletes;
//! the new outer face is the tool's UNNAMED cap, so `cube_PZ` would VANISH from
//! the result and every downstream `reference_selection` naming it would break.
//! The retired path only survived this via `setKernelSolid(result, { faceNames })` — a
//! geometric name RE-ASSOCIATION this port would otherwise have to reimplement.
//! `move_faces` keeps `cube_PZ` on the moved face byte-identically (same volume,
//! same face set on the planar case), so no re-association is needed. (If the
//! parent prefers the literal extrude+boolean replication, that re-association is
//! the "too involved" part the task authorizes skipping — but skipping it under
//! extrude+boolean silently breaks names, which is why this path was chosen.)
//!
//! Reference resolution (contract rule 1): `faces` is a `reference_selection` of
//! FACE names → `(handle, face_id)`; a miss is an `unresolved` entry, not a panic.
//!
//! Behaviour mapping — SOFT no-op (a warn +
//! `{ added: [], removed: [] }`) for empty selection, faces on multiple solids,
//! or a non-finite / zero distance; only a kernel failure throws. Matched here:
//! those cases return an empty result (no error); a `move_faces` `Err` (or a
//! non-planar / multi-loop face) → `ctx.fail` (halt).
//!
//! Name fidelity (contract rule 2): the result reuses
//! the TARGET solid's name and the target goes in `removed`; `move_faces`
//! preserves every face name, so names are collected, never re-stamped.
//!
//! Multi-loop planar faces (SM0): a planar carrier's normal is constant across its whole
//! domain, so a face with internal loops (a plate with a pocket or a drilled-then-capped
//! hole) pushes exactly like a single-loop one — `move_faces` already iterates every loop
//! and `retrim_planar_face` already re-trims all of them. Supported.
//!
//! Curved carriers use dedicated offset paths rather than translating a
//! midpoint normal: exact paths for cylinders/cones, spheres, and full tori;
//! residual-bounded paths for general revolutions and free-form NURBS.

use std::collections::HashSet;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureRefusal, FeatureResult};
use crate::{move_faces_unchecked, AnalyticSurface, BrepSolid, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // `faces` is a reference_selection of FACE names (contract rule 1).
    let names = common::reference_name_array(ctx.param("faces"));
    if names.is_empty() {
        return Ok(result); // Soft no-op — nothing selected.
    }

    // Resolve every face by EXACT name; misses recorded (not fatal).
    let mut resolved: Vec<(u32, u64)> = Vec::new();
    for name in &names {
        match ctx.scene.resolve_face(name) {
            Some(face_ref) => resolved.push((face_ref.handle, face_ref.face_id)),
            None => result.unresolved.push(name.clone()),
        }
    }
    if resolved.is_empty() {
        return Ok(result); // Only unresolved names — the caller repairs + re-dispatches.
    }

    // All selected faces must live on ONE solid (the caller soft-aborts otherwise).
    let handles: HashSet<u32> = resolved.iter().map(|(handle, _)| *handle).collect();
    if handles.len() > 1 {
        return Ok(result); // Soft no-op — faces from multiple solids.
    }
    let target_handle = resolved[0].0;

    // Distance: non-finite / ~zero → soft no-op.
    let distance = match ctx.param("distance") {
        None | Some(serde_json::Value::Null) => return Ok(result),
        Some(_) => ctx.number("distance")?,
    };
    if !distance.is_finite() || distance.abs() <= 1e-12 {
        return Ok(result);
    }
    // From here the feature PROCEEDS on the faces that resolved: a miss is
    // reported as a typed partial fulfilment beside `unresolved`, never
    // silently. (Not before the distance guard: a no-op applies nothing.)
    result.note_partial_resolution(&names);

    // Dedup face ids preserving order (a name listed twice, or two names → one id).
    let mut seen: HashSet<u64> = HashSet::new();
    let face_ids: Vec<u64> = resolved
        .iter()
        .map(|(_, face_id)| *face_id)
        .filter(|face_id| seen.insert(*face_id))
        .collect();

    // Result reuses the TARGET solid's name; reverse the
    // scene-map (one name per resident handle).
    let target_name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, handle)| **handle == target_handle)
        .map(|(name, _)| name.clone())
        .ok_or("push_face: target solid has no scene-map name")?;

    // Chain the pushes off a clone of the resident target. The outward normal is
    // recomputed from the CURRENT solid each pass (face ids are stable across
    // `move_faces`), so multiple faces push along their own up-to-date normals.
    let solid = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;

    let solid = push_faces(solid, &face_ids, distance, &mut result.notes, true)?;

    // `move_faces` preserves every face name — collect, never re-stamp.
    let face_names = common::collect_face_names(&solid);
    let edge_names = common::collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    result.added.push(AddedSolid {
        handle,
        name: target_name.clone(),
        face_names,
        edge_names,
        ..AddedSolid::default()
    });
    // The target is consumed; the loop reuses its name for the result.
    result.removed.push(target_name);
    Ok(result)
}

/// The push itself, shared with the public [`crate::offset_faces`]: each of
/// `face_ids` moved `distance` along its outward normal onto its offset
/// carrier (by carrier, as `build` documents), then the body's acceptance.
/// Notes the roads leave go to `notes_out`.
///
/// `past_neighbours`: whether a planar face carried BEYOND a neighbour (an
/// edge between them turned over) takes the re-cut road, the boolean the push
/// defines (the feature's answer), or is refused as ill-posed (the public
/// call's: the face pushed past the face it was to meet). A face landed
/// exactly ON a neighbour (an edge collapsed: two tops merged, a slot closed)
/// takes the re-cut road either way.
pub(crate) fn push_faces(
    mut solid: BrepSolid,
    face_ids: &[u64],
    distance: f64,
    notes_out: &mut Vec<String>,
    past_neighbours: bool,
) -> Result<BrepSolid, FeatureRefusal> {
    // A selected cylinder or cone written as SEVERAL faces (an imported bore
    // whose wall the exporter split into two half cylinders, its countersink
    // into two half cones) is one carrier the user picked whole. A half pushed
    // alone can never meet its unmoved sibling, so the selected pieces of one
    // carrier are first joined into the one face a whole-built bore has, and
    // that face is pushed once. Only faces of the selection join, only with
    // each other; a face that joins nothing is pushed as it was. The joined
    // face keeps the first piece's id and name.
    let mut face_ids = merge_selected_ruled_pieces(&mut solid, face_ids)?;
    for index in 0..face_ids.len() {
        let id = face_ids[index];
        let face_id = &id;
        // The faces still to push, followed into a re-cut body (which
        // numbers its faces afresh) where one is taken below.
        let mut followed: Option<Vec<u64>> = None;
        // Route by the pushed face's CARRIER (a face is a trimmed surface):
        // - PLANAR → translate the plane + re-intersect neighbours (`move_faces`,
        //   SM0–SM1c): the surface keeps its size.
        // - CYLINDER / CONE → OFFSET the carrier surface (the surface changes
        //   size — a pushed cylinder's radius grows) + re-intersect the neighbour
        //   caps (`offset_ruled_face`).
        // - SPHERE → concentric exact offset + neighbour re-trim.
        // - TORUS → exact minor-radius offset for an untrimmed full torus.
        // - REVOLUTION → residual-bounded meridian offset with planar healing.
        // - FREE-FORM → residual-bounded untrimmed NURBS offset.
        match face_carrier(&solid, *face_id)? {
            FaceCarrier::Planar => {
                // Positive distance pushes ALONG the outward normal (grow);
                // negative pushes inward (shrink).
                let normal = planar_face_normal(&solid, *face_id)?;
                solid = match move_faces_unchecked(&solid, &[*face_id], normal.scale(distance)) {
                    Ok(pushed) => pushed,
                    // A face beside a variable-radius fillet: the push runs on
                    // the body without the fillet, which is then re-made on the
                    // edge the push leaves, under its radius law. Only past a
                    // refusal, and only where such a fillet borders the face.
                    // A push that lands the face exactly on a neighbour, or
                    // carries it past one (a slot's wall pushed across the
                    // slot, closing it), has no re-solve: an edge collapses
                    // or turns over. Its answer is the boolean the push
                    // defines, so the RE-CUT road Transform Face takes is
                    // offered it (the body grown or cut by the face's swept
                    // slab, checked by the same acceptance); the re-solve's
                    // refusal stays the primary one if the re-cut declines.
                    Err(refusal) if !past_neighbours && is_inversion(&refusal) => {
                        return Err(crate::KernelRefusal::ill_posed(
                            crate::KernelStage::Refine,
                            "pushed_past_a_neighbour",
                            format!("offset_faces: the face is carried past a neighbour it was to meet ({refusal})"),
                        )
                        .into())
                    }
                    // Landed exactly on one neighbour, the push may still
                    // pass another (the move names the first edge it meets,
                    // in storage order): read short of the landing until a
                    // read lands on none, where only a face carried past a
                    // neighbour still turns an edge over.
                    Err(refusal)
                        if !past_neighbours
                            && is_collapse_or_inversion(&refusal)
                            && passes_a_neighbour_short_of(&solid, *face_id, normal, distance) =>
                    {
                        return Err(crate::KernelRefusal::ill_posed(
                            crate::KernelStage::Refine,
                            "pushed_past_a_neighbour",
                            format!("offset_faces: the face is carried past a neighbour it was to meet, landing on another ({refusal})"),
                        )
                        .into())
                    }
                    Err(refusal) if is_collapse_or_inversion(&refusal) => {
                        match super::transform_face::recut_past_a_refusal(&solid, &[*face_id], None, normal.scale(distance)) {
                            Ok((body, lost)) => {
                                notes_out.push(format!(
                                    "pushFace: the pushed face could not be re-solved ({refusal}), so it was \
                                     RE-CUT from the body instead: the boolean this push defines"
                                ));
                                notes_out.extend(lost);
                                followed = Some(follow_by_name(&solid, &body, &face_ids[index + 1..])?);
                                body
                            }
                            Err(reason) => {
                                let combined = refusal
                                    .with_message(|refusal| format!("{refusal}. The re-cut road was offered it too and declined: {reason}"));
                                return Err(super::transform_face::with_declines_class(combined, &[&reason]).into())
                            }
                        }
                    }
                    Err(refusal) => {
                        let without = crate::rebuild_displaced_blends(&solid, &[*face_id], "pushFace", |sharp| {
                            let pushed = move_faces_unchecked(sharp, &[*face_id], normal.scale(distance))?;
                            Ok((crate::accept_sound(pushed, "pushFace")?, Vec::new()))
                        });
                        match without {
                            None => return Err(refusal.into()),
                            Some(Err(decline)) => {
                                return Err(crate::with_blend_rebuild_decline(refusal, &decline).into())
                            }
                            Some(Ok((pushed, notes))) => {
                                notes_out.extend(notes);
                                pushed
                            }
                        }
                    }
                };
            }
            FaceCarrier::Ruled => {
                solid = match crate::offset_ruled_face_unchecked(&solid, *face_id, distance) {
                    Ok(pushed) => pushed,
                    Err(refusal) => return Err(name_unselected_sibling(&solid, *face_id, refusal).into()),
                };
            }
            FaceCarrier::Sphere => {
                solid = crate::offset_sphere_face_unchecked(&solid, *face_id, distance)?;
            }
            FaceCarrier::Torus => {
                solid = crate::offset_torus_face_unchecked(&solid, *face_id, distance)?;
            }
            FaceCarrier::Revolution => {
                solid = crate::offset_revolution_face_unchecked(&solid, *face_id, distance)?;
            }
            FaceCarrier::Freeform => {
                solid = crate::offset_freeform_face_unchecked(&solid, *face_id, distance)?;
            }
        }
        if let Some(rest) = followed {
            face_ids.truncate(index + 1);
            face_ids.extend(rest);
        }
    }

    // Pushing a carrier past a neighbour's own extent leaves the moved face
    // crossing one it should have stopped at, and pushing a concave one past
    // its curvature radius folds it through itself. Both bodies validate and
    // both measure a volume, so this is the one place the push is asked
    // whether its result is sound; it covers every carrier the dispatch above
    // routes to, including the planar `move_faces` lane.
    let solid = crate::accept_sound(solid, "pushFace")?;

    Ok(solid)
}

/// The faces `ids` of `before` in `after`, a re-cut of it that numbers its
/// faces afresh. Each is found by its name where that is one face's in both;
/// otherwise as the one face of `after` on the same carrier (through three of
/// its points), of the same sense, the same trimmed area (to 1e-9: a face
/// the re-cut split or trimmed is not it) and the same corners; a named match
/// must keep its corners too. A face found neither way cannot be
/// followed, and the push refuses by name rather than push some other face.
fn follow_by_name(before: &BrepSolid, after: &BrepSolid, ids: &[u64]) -> Result<Vec<u64>, crate::KernelRefusal> {
    fn faces(solid: &BrepSolid) -> Vec<&crate::FaceRecord> {
        solid.shells.iter().flat_map(|shell| &shell.faces).collect()
    }
    let (old, new) = (faces(before), faces(after));
    let scale = crate::solid_model_scale(before).max(1.0);
    let lost = |id: u64| {
        crate::KernelRefusal::unsupported(
            crate::KernelStage::Collect,
            "recut_lost_selection",
            format!("pushFace: a push was re-cut, which numbers the body's faces afresh, and face {id} still to push cannot be found in it"),
        )
    };
    let area = |face: &crate::FaceRecord| crate::face_area(face).ok();
    // A face's corners, sorted: an untouched face keeps them exactly where
    // they were, and two faces apart never share them all.
    let corners = |solid: &BrepSolid, face: &crate::FaceRecord| -> Vec<[f64; 3]> {
        let mut ids: Vec<u64> = face
            .loops
            .iter()
            .flat_map(|lp| &lp.coedges)
            .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .collect();
        ids.sort_unstable();
        ids.dedup();
        let mut points: Vec<[f64; 3]> = ids.iter().filter_map(|id| solid.vertices.iter().find(|v| v.id == *id)).map(|v| [v.point.x, v.point.y, v.point.z]).collect();
        points.sort_by(|a, b| a.iter().zip(b).map(|(x, y)| x.total_cmp(y)).find(|o| o.is_ne()).unwrap_or(std::cmp::Ordering::Equal));
        points
    };
    let same_corners = |a: &[[f64; 3]], b: &[[f64; 3]]| a.len() == b.len() && a.iter().zip(b).all(|(p, q)| (0..3).all(|k| (p[k] - q[k]).abs() <= 1e-9 * scale));
    ids.iter()
        .map(|&id| {
            let face = old.iter().find(|face| face.id == id).ok_or_else(|| lost(id))?;
            let was = corners(before, face);
            let kept = |other: &crate::FaceRecord| same_corners(&was, &corners(after, other));
            if let Some(name) = face.name.as_deref() {
                let named_old: Vec<&crate::FaceRecord> = old.iter().filter(|f| f.name.as_deref() == Some(name)).copied().collect();
                let named_new: Vec<&crate::FaceRecord> = new.iter().filter(|f| f.name.as_deref() == Some(name)).copied().collect();
                if let ([_], [found]) = (named_old.as_slice(), named_new.as_slice()) {
                    if kept(found) {
                        return Ok(found.id);
                    }
                }
            }
            let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else { return Err(lost(id)) };
            let probes: Vec<crate::Vec3> = [(0.25, 0.25), (0.75, 0.3), (0.4, 0.8)]
                .iter()
                .filter_map(|&(a, b)| face.surface.evaluate(u0 + a * (u1 - u0), v0 + b * (v1 - v0)).ok())
                .collect();
            let want = area(face).ok_or_else(|| lost(id))?;
            let matches: Vec<u64> = new
                .iter()
                .filter(|other| other.same_sense == face.same_sense)
                .filter(|other| probes.len() == 3 && probes.iter().all(|&p| crate::project_point_to_surface(&other.surface, p).map_or(false, |hit| hit.distance <= 1e-9 * scale)))
                .filter(|other| area(other).is_some_and(|got| (got - want).abs() <= 1e-9 * want.abs().max(1e-300)))
                .filter(|other| kept(other))
                .map(|other| other.id)
                .collect();
            match matches.as_slice() {
                [found] => Ok(*found),
                _ => Err(lost(id)),
            }
        })
        .collect()
}

/// Whether the planar push of `face_id` by `distance` along `normal` carries
/// the face past a neighbour before it lands on another: read short of the
/// landing by twice the move's own tolerance (its max(1e-7 of the model's
/// scale, 1e-9)), stepping back again while that read still lands on a
/// neighbour (another terrace at its height), until a read either builds (no
/// edge turned over there: nothing passed) or turns an edge over (a neighbour
/// passed). The move checks every edge, so the first read that does not land
/// answers whatever order the edges are stored in. A neighbour crossed by
/// less than three times the tolerance (its edge no longer than the move's
/// tolerance at the first read short of the landing, which the move reads as
/// landed) is within the move's resolution and is not read; past 16 landings
/// in a row it is answered as not passed.
fn passes_a_neighbour_short_of(solid: &BrepSolid, face_id: u64, normal: Vec3, distance: f64) -> bool {
    let step = 2.0 * (crate::solid_model_scale(solid) * 1e-7).max(1e-9);
    let mut short = distance;
    for _ in 0..16 {
        short -= step.copysign(distance);
        if short * distance <= 0.0 {
            return false;
        }
        match move_faces_unchecked(solid, &[face_id], normal.scale(short)) {
            Err(refusal) if is_inversion(&refusal) => return true,
            Err(refusal) if is_collapse_or_inversion(&refusal) => continue,
            _ => return false,
        }
    }
    false
}

/// The planar push's refusal for an edge it turns over: the face carried
/// past a neighbour.
fn is_inversion(refusal: &crate::KernelRefusal) -> bool {
    matches!(&refusal.class, crate::RefusalClass::UnsupportedGeometry { what } if what == crate::face_refusal_slugs::EDGE_INVERSION)
}

/// The planar push's refusal for an edge it collapses to nothing or turns
/// over: the face landed on, or carried past, a neighbour.
fn is_collapse_or_inversion(refusal: &crate::KernelRefusal) -> bool {
    matches!(&refusal.class, crate::RefusalClass::UnsupportedGeometry { what }
        if what == "edge_collapse" || what == crate::face_refusal_slugs::EDGE_INVERSION)
}

/// Join the selected cylinder/cone faces that are pieces of one carrier
/// (`crate::merge_faces_within`) and return the face ids to push: each
/// selected face replaced by the face now carrying it, in selection order,
/// deduplicated. With fewer than two ruled faces selected nothing is read and
/// the ids come back unchanged.
fn merge_selected_ruled_pieces(solid: &mut BrepSolid, face_ids: &[u64]) -> Result<Vec<u64>, crate::KernelRefusal> {
    let ruled: Vec<u64> = face_ids
        .iter()
        .copied()
        .filter(|id| matches!(face_carrier(solid, *id), Ok(FaceCarrier::Ruled)))
        .collect();
    if ruled.len() < 2 {
        return Ok(face_ids.to_vec());
    }
    let (merged, carried_by) = crate::merge_faces_within(solid, PIECE_MERGE_TOLERANCE, &ruled)?;
    *solid = merged;
    let mut seen: HashSet<u64> = HashSet::new();
    Ok(face_ids
        .iter()
        .map(|id| {
            carried_by
                .iter()
                .find(|(face, _)| face == id)
                .map(|(_, carrier)| *carrier)
                .unwrap_or(*id)
        })
        .filter(|id| seen.insert(*id))
        .collect())
}

/// The tolerance the face merge is given (the value its own solid-wide tests
/// use; its cosurface bands floor at `1e-7 x` the model's coordinates).
const PIECE_MERGE_TOLERANCE: f64 = 1e-7;

/// A ruled push that refused is re-read for its most common cause on imported
/// bodies: the face is ONE piece of a carrier split into several faces, and
/// its sibling was not selected. When an edge-adjacent unselected face joins
/// it under the same merge Push Face applies to a selection, the refusal names
/// that face and says to select it too; otherwise the original refusal stands.
fn name_unselected_sibling(solid: &BrepSolid, face_id: u64, refusal: crate::KernelRefusal) -> crate::KernelRefusal {
    let edges_of = |id: u64| -> HashSet<u64> {
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| face.id == id)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.edge_id)
            .collect()
    };
    let own = edges_of(face_id);
    let siblings: Vec<&crate::FaceRecord> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| {
            face.id != face_id
                && matches!(face.surface.analytic(), Some(AnalyticSurface::RuledRevolution { .. }))
                && !edges_of(face.id).is_disjoint(&own)
        })
        .collect();
    for sibling in siblings {
        let Ok((_, carried_by)) = crate::merge_faces_within(solid, PIECE_MERGE_TOLERANCE, &[face_id, sibling.id]) else {
            continue;
        };
        if carried_by.iter().all(|(_, carrier)| *carrier == carried_by[0].1) {
            let name = sibling.name.clone().unwrap_or_else(|| format!("face {}", sibling.id));
            return crate::KernelRefusal::input(
                crate::KernelStage::Collect,
                "split_carrier_piece",
                format!(
                    "push_face: face {face_id} is one piece of a cylinder/cone split into several faces; \
                     select its sibling `{name}` too so the whole carrier is pushed ({})",
                    refusal.message
                ),
            );
        }
    }
    refusal
}

/// The supported push carriers, dispatched on in `build`.
enum FaceCarrier {
    /// A plane — translate + re-intersect (`move_faces`).
    Planar,
    /// A cylinder or cone — offset the carrier + re-intersect (`offset_ruled_face`).
    Ruled,
    /// A sphere — offset to a concentric sphere + re-trim planar (through-centre)
    /// neighbours (`offset_sphere_face`).
    Sphere,
    /// A torus — offset the tube radius exactly while preserving both periods.
    Torus,
    /// A general full- or partial-sweep surface of revolution.
    Revolution,
    /// A non-analytic NURBS carrier; currently the untrimmed single-face case.
    Freeform,
}

/// Classify the pushed face's carrier. A datum/plane with no resident geometry
/// never reaches here (faces resolve to solids).
fn face_carrier(solid: &BrepSolid, face_id: u64) -> Result<FaceCarrier, String> {
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.id != face_id {
                continue;
            }
            return match face.surface.analytic() {
                Some(AnalyticSurface::Plane { .. }) => Ok(FaceCarrier::Planar),
                Some(AnalyticSurface::RuledRevolution { .. }) => Ok(FaceCarrier::Ruled),
                Some(AnalyticSurface::Sphere { .. }) => Ok(FaceCarrier::Sphere),
                Some(AnalyticSurface::Torus { .. }) => Ok(FaceCarrier::Torus),
                Some(AnalyticSurface::Revolution { .. }) => Ok(FaceCarrier::Revolution),
                None => Ok(FaceCarrier::Freeform),
            };
        }
    }
    Err(format!("push_face: face id {face_id} not found"))
}

/// The outward unit normal of a PLANAR face at its parameter-domain midpoint,
/// oriented by `same_sense` (mirror's `plane_from_face` pattern). Internal loops
/// (holes) are fine — a plane's normal is constant across the domain, so the
/// midpoint evaluation is valid even when it lands in a hole. Errors (deferred,
/// never wrong geometry) only on a non-planar carrier.
fn planar_face_normal(solid: &BrepSolid, face_id: u64) -> Result<Vec3, String> {
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.id != face_id {
                continue;
            }
            if !matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
                return Err(
                    "push_face: only planar faces are migrated (cylindrical radius-delta \
                     and other curved carriers are deferred)"
                        .into(),
                );
            }
            let [u0, u1] = face.surface.domain_u()?;
            let [v0, v1] = face.surface.domain_v()?;
            let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
            let mut normal = face.surface.normal(um, vm)?;
            if !face.same_sense {
                normal = normal.scale(-1.0);
            }
            return Ok(normal);
        }
    }
    Err(format!("push_face: face id {face_id} not found"))
}



/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "PF",
    "shortName": "PF",
    "longName": "Push Face",
    "ribbonPath": "Home/Synchronous Modeling/Push Face",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the push face feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "Select one or more faces on a single solid to push"
        },
        "distance": {
            "type": "number",
            "default_value": 1,
            "hint": "Signed distance to push the selected faces along their normals"
        }
    }
})
}

