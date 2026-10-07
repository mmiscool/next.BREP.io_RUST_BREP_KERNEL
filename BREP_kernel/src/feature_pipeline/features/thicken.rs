//! THK — Thicken. Grow one or more selected FACES of a prior solid into slab
//! solids, each bounded by the face and its offset plus ruled side walls.
//!
//! Semantics:
//!
//! - `inputParams.face` — a `reference_selection` (FACE names), `multiple`.
//!   Each selected face thickens into its OWN solid — or into SEVERAL, when a
//!   fold band across its trim is dropped and leaves a regular piece on either
//!   side of the band.
//! - `inputParams.distance` — the signed thickness along the face normal; NaN / 0
//!   is a soft no-op.
//!
//! Kernel path: the dedicated §5.9 sheet thickener
//! [`crate::thicken_trimmed_sheet`], fed the face's actual trim loops (the offset
//! shares the sheet basis, so a planar face thickens to an EXACT slab and a
//! circular hole grows an exact tube). The loop windings are normalized to the
//! outer-CCW / hole-CW convention the thickener requires; the thickness sign is
//! flipped for a `same_sense = false` face so the slab grows along the face's
//! OUTWARD normal (matching the established extrude-along-normal contract). A face whose
//! trim loops the thickener rejects (e.g. a general non-iso trim on a curved
//! sheet) REFUSES with that reason; it does not fall back to the full-domain
//! [`crate::thicken_face_sheet`], which would thicken the whole parameter
//! rectangle and ship a wrong-shape solid instead of the honest refusal.
//!
//! Naming (contract rule 2): a single selected face
//! names its solid `${featureId}`; with N > 1 faces each is
//! `${featureId}_${index:0width}_${sanitize(sourceFaceName)}` (width =
//! max(2, digits(N))).  A face that thickens into MORE THAN ONE body suffixes
//! each with `_${piece:0width}` on top of that, off the same rule. Per-face BUILD failures are collected and the feature
//! hard-errors ONLY if NOTHING thickened; a
//! reference miss records `unresolved` (contract rule 1) and is skipped.
//!
//! Naming RESIDUAL: the kernel's thickened solid leaves its faces UNNAMED
//! (`thicken_trimmed_sheet` stamps `name: None`), so the slab's individual faces
//! carry no names — the original source/offset/wall per-face labeling
//! is not replicated here.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::topology::{FaceRecord, LoopRecord};
use crate::{BrepSolid, NurbsCurve, NurbsSurface};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // Distance: `Number(distance)`; NaN / 0 warns + no-ops (not a throw).
    let distance = match ctx.number("distance") {
        Ok(distance) if distance.is_finite() && distance != 0.0 => distance,
        _ => return result,
    };

    let selections = common::unique_reference_names(ctx.param("face"));
    // Resolve BY NAME; a miss is a structured `unresolved` entry (rule 1), skipped.
    let mut faces: Vec<(String, crate::feature_pipeline::FaceRef)> = Vec::new();
    for name in &selections {
        match ctx.scene.resolve_face(name) {
            Some(face) => faces.push((name.clone(), face)),
            None => result.unresolved.push(name.clone()),
        }
    }
    if faces.is_empty() {
        // No valid face selections → the caller warns + returns empty (soft no-op).
        return result;
    }
    // The feature proceeds on the faces that resolved: a miss is reported as
    // a typed partial fulfilment beside `unresolved`, never silently.
    result.note_partial_resolution(&selections);

    let feature_id = if ctx.id.is_empty() {
        ctx.feature_type.clone()
    } else {
        ctx.id.clone()
    };
    let multiple = faces.len() > 1;
    let width = faces.len().to_string().len().max(2);

    let mut failures: Vec<String> = Vec::new();
    // The first face's refusal carries its class to the feature result; the
    // text stays the joined sentence over every face that failed.
    let mut first_refusal: Option<crate::KernelRefusal> = None;
    for (index, (face_name, face)) in faces.iter().enumerate() {
        let result_name = if multiple {
            format!(
                "{feature_id}_{:0>width$}_{}",
                index + 1,
                sanitize_token(face_name),
                width = width,
            )
        } else {
            feature_id.clone()
        };
        match thicken_face(face.handle, face.face_id, distance) {
            Ok(bodies) => {
                // ONE face can thicken into more than one body: a fold BAND
                // across its trim is dropped and the regular piece on either
                // side of the band is its own slab, sharing no edge with the
                // other. They are numbered off the same `_{index:0width}` shape
                // the multi-FACE rule uses, so a document that selected one face
                // and got two bodies still names them predictably.
                let bodies_width = bodies.len().to_string().len().max(2);
                let split = bodies.len() > 1;
                for (piece, solid) in bodies.into_iter().enumerate() {
                    let face_names = common::collect_face_names(&solid);
                    let edge_names = common::collect_edge_names(&solid);
                    let handle = crate::register_solid_value(solid);
                    result.added.push(AddedSolid {
                        handle,
                        name: if split {
                            format!(
                                "{result_name}_{:0>width$}",
                                piece + 1,
                                width = bodies_width
                            )
                        } else {
                            result_name.clone()
                        },
                        face_names,
                        edge_names,
                        ..AddedSolid::default()
                    });
                }
            }
            Err(error) => {
                failures.push(format!("{face_name}: {error}"));
                first_refusal.get_or_insert(error);
            }
        }
    }

    // Hard error only when NOTHING was produced.
    if result.added.is_empty() {
        let message = format!(
            "Thicken failed to produce any solids: {}",
            failures.join("; ")
        );
        return match first_refusal {
            Some(refusal) => ctx.fail(refusal.with_message(|_| message)),
            None => ctx.fail(message),
        };
    }
    result
}

/// Token sanitization: disallowed characters collapse to
/// `_`; empty → `FACE`. (Faithful per-char; the original additionally collapses RUNS of
/// `:[]`/whitespace to a single `_` — a minor residual for names with such runs.)
fn sanitize_token(value: &str) -> String {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return "FACE".to_string();
    }
    let sanitized: String = trimmed
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || ch == '_' || ch == '.' || ch == '-' {
                ch
            } else {
                '_'
            }
        })
        .collect();
    if sanitized.is_empty() {
        "FACE".to_string()
    } else {
        sanitized
    }
}

/// Thicken one face (by resident handle + face id) into a slab solid.
fn thicken_face(handle: u32, face_id: u64, distance: f64) -> Result<Vec<BrepSolid>, crate::KernelRefusal> {
    crate::with_registered_solid_typed(handle, |solid| {
        let face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_id)
            .ok_or_else(|| {
                crate::KernelRefusal::internal(
                    crate::KernelStage::Collect,
                    "thicken_face_not_found",
                    format!("thicken: face {face_id} not found on solid"),
                )
            })?;
        // Grow along the face OUTWARD normal: the thickener's normal is Su×Sv, the
        // face normal is that only when `same_sense`, so flip the sign otherwise.
        let thickness = if face.same_sense { distance } else { -distance };
        // Thicken the face's REAL trim region, and refuse if that is not
        // supported.  There used to be a fallback here to the full-domain
        // `thicken_face_sheet` whenever the trimmed path errored — but that
        // thickens the WHOLE parameter rectangle, so for a genuinely trimmed
        // curved face it shipped a wrong-SHAPE solid in place of the kernel's
        // honest refusal.  That breaks the offset-shell work's standing rule —
        // never weaken the honesty gate, because a wrong-but-watertight result
        // is worse than a refusal — and it swallowed the
        // one refusal that tells a caller the general-trim capability is
        // missing (`offset/thicken.rs` non-iso trim on a curved sheet).
        // Propagate the real reason instead: a capability gap must present as
        // a refusal, never as a different solid.
        let loops = crate::OrRefuse::or_refuse(build_trim_loops(face), crate::KernelStage::Collect, "build_trim_loops")?;
        crate::thicken_trimmed_sheet(&face.surface, &loops, thickness, false)
    })
}

/// The face's trim loops as `thicken_trimmed_sheet` wants them: outer loop first,
/// wound counter-clockwise in (u, v), hole loops clockwise. The stored
/// `coedge.pcurve` is already oriented along the loop traversal (verified by
/// `parameter_space_area`, which walks pcurves natively), so a loop is just its
/// pcurves in order; winding is normalized against the OUTER loop's signed area.
fn build_trim_loops(face: &FaceRecord) -> Result<Vec<Vec<NurbsCurve>>, String> {
    if face.loops.is_empty() {
        return Err("thicken: face has no trim loops".into());
    }
    // Order loops so the largest-|area| (outer) loop is first.
    let mut ordered: Vec<(usize, f64)> = Vec::with_capacity(face.loops.len());
    for (index, loop_record) in face.loops.iter().enumerate() {
        ordered.push((index, loop_signed_area(&face.surface, loop_record)?));
    }
    ordered.sort_by(|a, b| b.1.abs().partial_cmp(&a.1.abs()).unwrap_or(std::cmp::Ordering::Equal));
    // If the outer loop is clockwise, flip EVERY loop together (outer → CCW,
    // holes → CW) so the whole trim matches the thickener's convention.
    let reverse = ordered[0].1 < 0.0;
    let mut loops = Vec::with_capacity(face.loops.len());
    for (index, _) in &ordered {
        let curves: Vec<NurbsCurve> = face.loops[*index]
            .coedges
            .iter()
            .map(|coedge| coedge.pcurve.clone())
            .collect();
        loops.push(if reverse { reverse_loop(&curves)? } else { curves });
    }
    Ok(loops)
}

/// Reverse a loop's traversal: reverse the pcurve order AND each pcurve.
fn reverse_loop(curves: &[NurbsCurve]) -> Result<Vec<NurbsCurve>, String> {
    curves.iter().rev().map(|curve| curve.reversed()).collect()
}

/// Signed (u, v) area of one loop (Green's theorem over its pcurves).
/// `parameter_space_area` reads only the coedge pcurves, so any surface serves.
fn loop_signed_area(surface: &NurbsSurface, loop_record: &LoopRecord) -> Result<f64, String> {
    let face = FaceRecord {
        id: 0,
        surface: surface.clone(),
        same_sense: true,
        loops: vec![loop_record.clone()],
        name: None,
    };
    crate::parameter_space_area(&face)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected face drives `face`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "THK",
    "shortName": "THK",
    "longName": "Thicken",
    "ribbonPath": "Home/Modeling/Thicken",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the thicken feature."
        },
        "face": {
            "type": "reference_selection",
            "label": "Faces",
            "selectionFilter": [
                "FACE"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select one or more open faces to thicken into individual solids."
        },
        "distance": {
            "type": "number",
            "default_value": 1,
            "hint": "Signed thickness to apply along the face normals."
        }
    }
})
}

