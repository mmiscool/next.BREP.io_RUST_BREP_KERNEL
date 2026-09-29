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
use crate::{move_faces, AnalyticSurface, BrepSolid, Vec3};

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
    let mut solid = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;
    for face_id in &face_ids {
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
                solid = move_faces(&solid, &[*face_id], normal.scale(distance))?;
            }
            FaceCarrier::Ruled => {
                solid = crate::offset_ruled_face(&solid, *face_id, distance)?;
            }
            FaceCarrier::Sphere => {
                solid = crate::offset_sphere_face(&solid, *face_id, distance)?;
            }
            FaceCarrier::Torus => {
                solid = crate::offset_torus_face(&solid, *face_id, distance)?;
            }
            FaceCarrier::Revolution => {
                solid = crate::offset_revolution_face(&solid, *face_id, distance)?;
            }
            FaceCarrier::Freeform => {
                solid = crate::offset_freeform_face(&solid, *face_id, distance)?;
            }
        }
    }

    // Pushing a carrier past a neighbour's own extent leaves the moved face
    // crossing one it should have stopped at, and pushing a concave one past
    // its curvature radius folds it through itself. Both bodies validate and
    // both measure a volume, so this is the one place the push is asked
    // whether its result is sound; it covers every carrier the dispatch above
    // routes to, including the planar `move_faces` lane.
    let solid = crate::accept_sound(solid, "pushFace")?;

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

