//! TF — Transform Face.
//!
//! Carries the selected faces of a single solid by ONE RIGID MOTION — a rotation
//! about a stated pivot followed by a translation — preserving each carrier's
//! shape, and re-intersects the neighbours the motion moves the selection
//! relative to. The motion is the `transform` parameter the transform gizmo
//! edits: dragging an axis or the centre handle writes `position`, dragging a
//! ring writes `rotationEuler`.
//!
//! # The motion
//!
//! `p ↦ pivot + position + R·(p − pivot)`, with `R` the intrinsic XYZ Euler
//! rotation of `rotationEuler` (degrees) — exactly
//! [`common::compose_trs_matrix`]`(position, rotationEuler, [1,1,1], pivot)`, the
//! one feature-transform convention the Transform feature, the assembly pose and
//! the gizmo already speak. The gizmo is drawn at `pivot + position`, which is
//! where the pivot point lands, so a ring drag turns the faces about the handle
//! the user is holding.
//!
//! `scale` has no meaning for a face edit — a scaled carrier is a different
//! surface, not a moved one. The schema's `default_value` omits it, which hides
//! it in the form and in the tool schema, and a document that carries a
//! non-unit scale anyway is refused BY NAME rather than silently ignored.
//!
//! # A rotation is not a mode of a translation — and this feature does not pretend it is
//!
//! [`crate::move_faces`] moves a carrier's POSITION and leaves its normal alone;
//! [`crate::rotate_faces`] moves the normal too, so the neighbour pairs are
//! re-intersected differently and the invariants that let a corner ride rigidly
//! are different invariants (a cone about its own axis is invariant under a
//! rotation and not under a translation). The two primitives stay two roads, and
//! this feature is a DISPATCHER over them, not a third solver:
//!
//! * a pure TRANSLATION (`rotationEuler` is the identity) goes to `move_faces`
//!   with `position` — Move Face's road, unchanged;
//! * a pure ROTATION (`position` is zero) goes to `rotate_faces` about the axis
//!   LINE through `pivot` along the rotation's own axis, by its own angle —
//!   Rotate Face's road, unchanged. `R` is converted to that axis and angle from
//!   the same matrix the compose builds, so the stored Euler angles and the
//!   kernel's Rodrigues rotation are one rotation;
//! * a COMBINED motion is rotate-then-translate: `rotate_faces` about the pivot,
//!   then `move_faces` by `position` on the solid the rotation left. Both are
//!   pure geometry edits that keep every face id, so the second call takes the
//!   same ids the first did.
//!
//! Why not one rigid re-intersection: neither primitive accepts a general rigid
//! motion. `move_faces` takes a vector and builds its rim maps from it (the
//! exact affine for a ruled carrier, the plane × sphere circle); `rotate_faces`
//! takes an axis and an angle and tests carrier invariance against that axis
//! line. A third road accepting a matrix would have to redo both invariance
//! analyses per case, which is precisely the objection the Rotate Face contract
//! raised against a matrix: a matrix names no invariant. Decomposing keeps the
//! invariant each road proves.
//!
//! Why not the screw decomposition (Chasles: any rigid motion is a rotation
//! about SOME axis plus a slide along it, which would fold the in-plane part of
//! `position` into a single `rotate_faces` call): that axis sits at
//! `|position⊥| / (2·sin(θ/2))` from the pivot — metres away for a degree of turn
//! — and it is no longer the selection's own axis, so a coaxial cylinder spun
//! and slid would lose the rigid regime it has about the pivot. The pivot the
//! user stated is the axis the invariance test must see.
//!
//! The price of two calls is an INTERMEDIATE state: the faces rotated but not yet
//! translated. A motion whose end state is buildable can still refuse there (a
//! rotation that alone inverts a neighbour the translation would have restored).
//! Such a refusal is named as the step it happened in — "the rotation step" or
//! "the translation step" — with the primitive's own message after it, so the
//! user reads which half of the motion to change.
//!
//! # The pivot
//!
//! `pivot` is a stored vec3. When it is null the pivot is the SELECTION'S CENTRE,
//! computed by [`selection_centre`] from the feature's INPUT solid: the centre of
//! the axis-aligned bounds of every vertex on the selected faces' boundaries and
//! of each boundary edge sampled at [`CENTRE_SAMPLES`] equal steps across its
//! active parameter range (a face with no edge at all — a closed sphere —
//! contributes its surface control points). Exact for straight-edged faces.
//! The app stores that value into `pivot` the moment the face selection is
//! committed (through [`face_transform_pivot`], the same computation), so the
//! rotation centre is a number in the document: re-running the history, dragging
//! the gizmo, or editing a feature upstream never moves it. A null pivot in a
//! hand-written document is resolved the same way on every run, from the same
//! input, so it cannot drift either.
//!
//! # Reference resolution and behaviour mapping
//!
//! `faces` is a `reference_selection` of FACE names — the declaration Push Face
//! and Delete Face use. A miss is an `unresolved` entry, never a panic. Every
//! `transform` component and every `pivot` component may be a number or an
//! expression.
//!
//! SOFT no-op (an empty result, no error) for an empty selection, names that all
//! fail to resolve, faces spanning multiple solids, and the IDENTITY motion — a
//! half-filled dialog must not shout. A kernel `Err` → `ctx.fail` (halt): the
//! refusal names the offending entity, and neither primitive mutates its input,
//! so the source solid is untouched.
//!
//! # Name preservation and the exit
//!
//! Both roads are pure geometry edits: no face, edge or vertex is created or
//! destroyed, so every id and name survives. Names are COLLECTED, never
//! re-stamped, and the result reuses the TARGET solid's name. The result goes
//! through [`crate::accept_sound`] once, at the exit, as Move Face and Rotate
//! Face each did.
//!
//! The acceptance is the one step that can change topology, and only on a
//! BREAKOUT — a motion that carries the selection clean out of the faces it
//! opened through, which both primitives build as a body passing through
//! itself. Its crossing repair then splits the pair along their carriers' exact
//! section, drops what ended up on the wrong side and RE-TRIMS the neighbour
//! that is left holding an unanswered boundary, so edges and vertices move
//! while every FACE name still survives. It says so in the result's `notes`
//! rather than changing the answer silently.

use crate::face_refusal_slugs::{PIVOT_GEOMETRY, RIGID_SCALE};
use std::collections::HashSet;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureRefusal, FeatureResult, SceneMap};
use crate::{move_faces, rotate_faces, BrepSolid, KernelRefusal, KernelStage, Vec3};

/// Stations per boundary edge in [`selection_centre`]. A power of two, so a
/// rational circle split into quarter spans is sampled at its span ends — the
/// points where an axis-aligned circle reaches its bounds.
/// [`FeatureResult::refused_step`] of a Transform Face whose ROTATION refused
/// (a turn alone, or the first half of rotate-then-translate).
pub const REFUSED_STEP_ROTATION: &str = "rotation";
/// … whose TRANSLATION refused (a move alone, or the second half).
pub const REFUSED_STEP_TRANSLATION: &str = "translation";

pub const CENTRE_SAMPLES: usize = 64;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// The selection resolved against a scene: the one target solid's handle and
/// the deduplicated face ids on it, or `None` for the soft no-ops (nothing
/// resolved, or faces on more than one solid). Misses land in `unresolved`.
fn resolve_selection(
    scene: &SceneMap,
    names: &[String],
    unresolved: &mut Vec<String>,
) -> Option<(u32, Vec<u64>)> {
    let mut resolved: Vec<(u32, u64)> = Vec::new();
    for name in names {
        match scene.resolve_face(name) {
            Some(face_ref) => resolved.push((face_ref.handle, face_ref.face_id)),
            None => unresolved.push(name.clone()),
        }
    }
    let &(target, _) = resolved.first()?;
    if resolved.iter().any(|(handle, _)| *handle != target) {
        return None;
    }
    let mut seen: HashSet<u64> = HashSet::new();
    let face_ids = resolved
        .iter()
        .map(|(_, face_id)| *face_id)
        .filter(|face_id| seen.insert(*face_id))
        .collect();
    Some((target, face_ids))
}

/// The centre of the selection's boundary bounds — the default pivot. See the
/// module doc for exactly what is bounded. `None` only when the faces carry no
/// geometry at all.
pub fn selection_centre(solid: &BrepSolid, face_ids: &[u64]) -> Option<Vec3> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    let mut include = |point: Vec3| {
        let point = [point.x, point.y, point.z];
        if point.iter().all(|c| c.is_finite()) {
            for axis in 0..3 {
                low[axis] = low[axis].min(point[axis]);
                high[axis] = high[axis].max(point[axis]);
            }
        }
    };
    let mut edge_ids: HashSet<u64> = HashSet::new();
    let mut edgeless = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if !selected.contains(&face.id) {
            continue;
        }
        let before = edge_ids.len();
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            edge_ids.insert(coedge.edge_id);
        }
        if edge_ids.len() == before && face.loops.iter().all(|l| l.coedges.is_empty()) {
            edgeless.push(face);
        }
    }
    let vertex_ids: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edge_ids.contains(&edge.id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    for vertex in &solid.vertices {
        if vertex_ids.contains(&vertex.id) {
            include(vertex.point);
        }
    }
    for edge in solid.edges.iter().filter(|edge| edge_ids.contains(&edge.id)) {
        if edge.degenerate {
            continue;
        }
        for station in 0..=CENTRE_SAMPLES {
            let t = edge.t0 + (edge.t1 - edge.t0) * station as f64 / CENTRE_SAMPLES as f64;
            if let Ok(point) = edge.curve.evaluate(t) {
                include(point);
            }
        }
    }
    for face in edgeless {
        for control in face.surface.control_points.iter().flatten() {
            if let Ok(point) = control.point() {
                include(point);
            }
        }
    }
    if !low[0].is_finite() {
        return None;
    }
    Some(Vec3::new(
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    ))
}

/// The pivot a Transform Face whose `faces` are `names` gets by default, read
/// off the scene the results BEFORE it build — the value the app stores into
/// `pivot` when the selection is committed, and the value `build` falls back to
/// for a null `pivot`, because both go through [`resolve_selection`] and
/// [`selection_centre`]. `None` when the selection does not resolve to faces on
/// one solid.
pub fn face_transform_pivot(prefix: &[FeatureResult], names: &[String]) -> Option<[f64; 3]> {
    let mut scene = SceneMap::default();
    for result in prefix {
        scene.apply(result);
    }
    let (handle, face_ids) = resolve_selection(&scene, names, &mut Vec::new())?;
    let centre =
        crate::with_registered_solid_str(handle, |solid| Ok(selection_centre(solid, &face_ids)))
            .ok()??;
    Some([centre.x, centre.y, centre.z])
}

/// The rotation part of the motion as the axis and angle `rotate_faces` takes:
/// `R` is built by the shared compose (so the Euler convention is single-sourced)
/// and converted through its quaternion, choosing the numerically largest
/// component first so a half turn keeps its axis. `None` for the identity.
pub fn rotation_axis_angle(rotation_deg: [f64; 3]) -> Option<(Vec3, f64)> {
    if rotation_deg == [0.0; 3] {
        return None;
    }
    let radians = rotation_deg.map(f64::to_radians);
    let m = common::compose_trs_matrix([0.0; 3], radians, [1.0; 3], [0.0; 3]);
    let at = |row: usize, col: usize| m[row * 4 + col];
    let (m00, m11, m22) = (at(0, 0), at(1, 1), at(2, 2));
    let trace = m00 + m11 + m22;
    // [x, y, z, w]
    let q = if trace >= m00 && trace >= m11 && trace >= m22 {
        let w = 0.5 * (1.0 + trace).max(0.0).sqrt();
        let k = 0.25 / w;
        [(at(2, 1) - at(1, 2)) * k, (at(0, 2) - at(2, 0)) * k, (at(1, 0) - at(0, 1)) * k, w]
    } else if m00 >= m11 && m00 >= m22 {
        let x = 0.5 * (1.0 + m00 - m11 - m22).max(0.0).sqrt();
        let k = 0.25 / x;
        [x, (at(0, 1) + at(1, 0)) * k, (at(0, 2) + at(2, 0)) * k, (at(2, 1) - at(1, 2)) * k]
    } else if m11 >= m22 {
        let y = 0.5 * (1.0 - m00 + m11 - m22).max(0.0).sqrt();
        let k = 0.25 / y;
        [(at(0, 1) + at(1, 0)) * k, y, (at(1, 2) + at(2, 1)) * k, (at(0, 2) - at(2, 0)) * k]
    } else {
        let z = 0.5 * (1.0 - m00 - m11 + m22).max(0.0).sqrt();
        let k = 0.25 / z;
        [(at(0, 2) + at(2, 0)) * k, (at(1, 2) + at(2, 1)) * k, z, (at(1, 0) - at(0, 1)) * k]
    };
    // One sign for the quaternion, so the angle lands in [0, π].
    let sign = if q[3] < 0.0 { -1.0 } else { 1.0 };
    let axis = Vec3::new(q[0] * sign, q[1] * sign, q[2] * sign);
    let half_sine = axis.length();
    let angle = 2.0 * half_sine.atan2(q[3] * sign);
    if angle <= 1e-12 {
        return None;
    }
    Some((axis.scale(1.0 / half_sine), angle))
}

/// A `transform` component, read through the shared vec3 reader under its
/// dotted name so an expression error names `transform.position` and not a bare
/// key.
fn transform_vec3(ctx: &FeatureContext, key: &str, default: [f64; 3]) -> Result<[f64; 3], String> {
    common::vec3_from_value(
        ctx.env,
        ctx.param("transform").and_then(|transform| transform.get(key)),
        &format!("transform.{key}"),
        default,
    )
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    let names = common::reference_name_array(ctx.param("faces"));
    if names.is_empty() {
        return Ok(result); // Soft no-op — nothing selected.
    }
    let Some((target_handle, face_ids)) =
        resolve_selection(ctx.scene, &names, &mut result.unresolved)
    else {
        return Ok(result); // Only unresolved names, or faces on several solids.
    };
    // The feature proceeds on the faces that resolved: a miss is reported as
    // a typed partial fulfilment beside `unresolved`, never silently.
    result.note_partial_resolution(&names);

    // The motion. Scale first: a document that scales is refused before
    // anything else is read, because no reading of the rest makes it rigid.
    let scale = transform_vec3(ctx, "scale", [1.0; 3])?;
    if scale != [1.0; 3] {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            RIGID_SCALE,
            format!(
                "transformFace: `transform.scale` is {scale:?}, but a face transform is a RIGID \
                 motion — a scaled carrier is a different surface, not a moved one — so scale must \
                 be [1, 1, 1]; refusing"
            ),
        )
        .into());
    }
    let position = transform_vec3(ctx, "position", [0.0; 3])?;
    let rotation_deg = transform_vec3(ctx, "rotationEuler", [0.0; 3])?;
    let translation = Vec3::new(position[0], position[1], position[2]);
    let translates = translation.length() > 1e-12;
    let rotation = rotation_axis_angle(rotation_deg);
    if !translates && rotation.is_none() {
        return Ok(result); // Soft no-op — the identity motion.
    }

    let target_name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, handle)| **handle == target_handle)
        .map(|(name, _)| name.clone())
        .ok_or("transformFace: target solid has no scene-map name")?;
    let solid = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;

    // The rotation as the re-cut road needs it too: axis point, axis, angle.
    let turn = match rotation {
        None => None,
        Some((axis, angle)) => {
            let pivot = match ctx.param("pivot") {
                None | Some(serde_json::Value::Null) => selection_centre(&solid, &face_ids)
                    .ok_or_else(|| {
                        KernelRefusal::input(
                            KernelStage::Collect,
                            PIVOT_GEOMETRY,
                            "transformFace: the selected faces carry no geometry to centre a pivot on",
                        )
                    })?,
                Some(value) => {
                    let [x, y, z] = common::vec3_from_value(ctx.env, Some(value), "pivot", [0.0; 3])?;
                    Vec3::new(x, y, z)
                }
            };
            Some((pivot, axis, angle))
        }
    };
    // Every motion step records itself on refusal (`refused_step`), so the
    // app names the motion the user asked for without reading this text.
    let refused_at = |error: KernelRefusal, step: &str| {
        let mut refused = ctx.fail(error);
        refused.refused_step = Some(step.to_string());
        refused
    };
    let moved = match turn {
        None => match move_faces(&solid, &face_ids, translation) {
            Ok(moved) => moved,
            Err(error) => return Ok(refused_at(error, REFUSED_STEP_TRANSLATION)),
        },
        Some((pivot, axis, angle)) => {
            if !translates {
                match rotate_faces(&solid, &face_ids, pivot, axis, angle) {
                    Ok(turned) => turned,
                    Err(error) => return Ok(refused_at(error, REFUSED_STEP_ROTATION)),
                }
            } else {
                // ROTATE-THEN-TRANSLATE, each step named on refusal (module doc).
                // The primitive's class rides through the step's wrap.
                // The refused step is also RECORDED on the result
                // (`FeatureResult::refused_step`), so a reader need not parse it
                // out of this text.
                let turned = match rotate_faces(&solid, &face_ids, pivot, axis, angle) {
                    Ok(turned) => turned,
                    Err(error) => {
                        let error = error.with_message(|error| {
                            format!(
                                "transformFace: the ROTATION step of this rotate-then-translate motion \
                                 refused — {error}"
                            )
                        });
                        return Ok(refused_at(error, REFUSED_STEP_ROTATION));
                    }
                };
                match move_faces(&turned, &face_ids, translation) {
                    Ok(moved) => moved,
                    Err(error) => {
                        let error = error.with_message(|error| {
                            format!(
                                "transformFace: the TRANSLATION step of this rotate-then-translate \
                                 motion, applied to the faces as the rotation left them, refused — {error}"
                            )
                        });
                        return Ok(refused_at(error, REFUSED_STEP_TRANSLATION));
                    }
                }
            }
        }
    };
    // Carrying a face past a neighbour's own extent leaves it crossing one it
    // should have stopped at, and both roads are pure geometry edits that create
    // no cell for `validate()` to catch. The acceptance Push Face takes: split and
    // trim where that can be proved, refuse by name where it cannot. Where the
    // motion carried the selection clean OUT of the faces it opened through —
    // the breakout — that split also re-trims the neighbour left holding an
    // unanswered boundary, which is the only way such a body closes.
    //
    // And where the acceptance refuses, the RE-CUT road is offered the same
    // edit from the other end — the original body, the selection and the
    // motion, cut as a boolean rather than re-solved — because a motion whose
    // answer needs a face to STOP EXISTING has no body for a repair to reach.
    // A plane moved rigidly is still a plane, so the road takes a
    // rotate-then-translate as well as a translation (the 2026-09-22 slot
    // document with its wall also TURNED 16.9° refused here while the same
    // drag without the turn built). See `edit/direct_edit/face_move_recut.rs`.
    // It runs only after a refusal, so no motion that builds today changes by a
    // bit, and a motion `move_faces` / `rotate_faces` itself refused never
    // reaches it.
    let moved = match crate::accept_sound(moved, "transformFace") {
        Ok(body) => body,
        Err(refusal) => {
            match crate::recut_moved_planes_rigid(&solid, &face_ids, turn, translation) {
                Ok(body) => {
                    result.notes.push(format!(
                        "transformFace: the moved carriers could not be re-solved onto the body \
                         they left ({refusal}), so the selection was RE-CUT from the original \
                         body instead — the result is the boolean this motion defines, and a \
                         face whose carrier left the body is no longer on it"
                    ));
                    body
                }
                // The acceptance's refusal stays the primary one — its class
                // is what the boundary reports — and the re-cut's decline is
                // appended as context.
                Err(reason) => {
                    return Err(refusal
                        .with_message(|refusal| {
                            format!(
                                "{refusal}. The re-cut road was offered it too and declined: {reason}"
                            )
                        })
                        .into())
                }
            }
        }
    };

    // Both roads preserve every name — collect, never re-stamp.
    let face_names = common::collect_face_names(&moved);
    let edge_names = common::collect_edge_names(&moved);
    let handle = crate::register_solid_value(moved);
    result.added.push(AddedSolid {
        handle,
        name: target_name.clone(),
        face_names,
        edge_names,
        ..AddedSolid::default()
    });
    result.removed.push(target_name);
    Ok(result)
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "TF",
    "shortName": "TF",
    "longName": "Transform Face",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the transform face feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "Select one or more faces on a single solid to move and turn"
        },
        "pivot": {
            "type": "vec3",
            "label": "Pivot",
            "default_value": null,
            "hint": "The point the rotation turns about, in world millimetres; set to the centre of the selection when the faces are picked"
        },
        "transform": {
            "type": "transform",
            "default_value": {
                "position": [0, 0, 0],
                "rotationEuler": [0, 0, 0]
            },
            "hint": "Rotate the faces about the pivot, then move them; drag the gizmo or type the values"
        }
    }
})
}

