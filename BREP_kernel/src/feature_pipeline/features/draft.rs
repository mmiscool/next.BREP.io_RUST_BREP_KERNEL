//! DRAFT — Draft. Tilts chosen faces of a solid so the part can be pulled out
//! of a mould: the kernel's [`crate::draft_faces`] (neutral plane) and
//! [`crate::draft_faces_parted_sided`] (parting line) behind a form.
//!
//! The inputs (schema below; docs/features/draft.md says it for users):
//! - `faces` — the faces to draft (FACE, multiple). Each must be a plane that
//!   runs along the pull or a cylinder about it (a cylinder becomes a cone);
//!   the kernel refuses anything else by name.
//! - `type` — `Neutral plane` (faces turn about where they cross the neutral
//!   plane) or `Parting line` (the body is split at the plane and each side
//!   drafted away from it).
//! - `neutralPlane` — a planar face or a datum/construction plane: the plane
//!   that keeps its size, or the parting plane. Required.
//! - `pullDirection` / `flipPull` — the pull. By default it is the neutral
//!   plane's normal: a datum plane's own normal, and for a face of the body
//!   the normal pointing INTO the body (the face's outward normal reversed),
//!   so the picked face is where the part is largest and a positive angle
//!   leans the walls inward as they go away from it. A pull reference
//!   overrides it (a flat face gives its outward normal, a plane its own, an
//!   edge or axis its direction); it must be square to the neutral plane — drafting about a
//!   neutral plane that is not square to the pull is stage 2, refused by name.
//! - `angle` (degrees, expression) — positive leans each face into the
//!   material as it goes along the pull; negative leans it out. For a parting
//!   line, `angle` is the `+pull` side's and `angle2` the other side's unless
//!   `sameBothSides`.
//! - `faceSet` — `Selected` (the `faces` field), `All faces along the pull`
//!   (every face of the body the draft can take), `Outer` / `Inner` (the
//!   faces standing on the neutral FACE's outer boundary / on its holes, as
//!   SolidWorks' face propagation reads them; a datum plane has no boundary, so
//!   those two refuse by name there). Stored as the choice and resolved to
//!   faces at every build, so it follows the model.
//! - `tangentPropagation` — add the faces joined smoothly to the chosen ones
//!   that the draft can take ([`crate::draft_tangent_faces`]).
//!
//! The resolved pull and neutral point (numbers only, no names: the history
//! reads every string in `persistentData` as a reference) are published in the
//! result's `persistentData.draft`; the viewport draws the pull arrow and the
//! angle from them. The faces the set or propagation added are listed in a
//! note. The body keeps its name and every face its name (a face a parting
//! line cuts comes out as two, the second named as a split face, `_1`).

use std::collections::{HashMap, HashSet};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::{BrepSolid, KernelRefusal, KernelStage, Vec3};

/// The `type` choices.
pub const TYPE_NEUTRAL: &str = "Neutral plane";
pub const TYPE_PARTING: &str = "Parting line";
/// The `faceSet` choices.
pub const SET_SELECTED: &str = "Selected";
pub const SET_ALL: &str = "All faces along the pull";
pub const SET_OUTER: &str = "Outer";
pub const SET_INNER: &str = "Inner";

const OP: &str = "Draft";

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn refuse(slug: &str, message: String) -> FeatureRefusal {
    KernelRefusal::input(KernelStage::Collect, slug, format!("{OP}: {message}")).into()
}

fn flag(ctx: &FeatureContext, key: &str, default: bool) -> bool {
    ctx.param(key).and_then(serde_json::Value::as_bool).unwrap_or(default)
}

fn choice(ctx: &FeatureContext, key: &str, default: &str) -> String {
    ctx.param(key)
        .and_then(serde_json::Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .unwrap_or(default)
        .to_string()
}

fn angle_degrees(ctx: &FeatureContext, key: &str) -> Result<f64, FeatureRefusal> {
    let value = ctx.number(key).map_err(|error| refuse("draft_angle", error))?;
    if !value.is_finite() {
        return Err(refuse("draft_angle", format!("`{key}` must be a finite angle in degrees")));
    }
    Ok(value)
}

/// The neutral plane as `(point, unit normal, the neutral FACE if it is one)`.
/// The normal is the default pull: a datum plane's own normal, a body face's
/// inward normal.
fn neutral_plane(ctx: &FeatureContext, result: &mut FeatureResult) -> Result<(Vec3, Vec3, Option<crate::feature_pipeline::FaceRef>), FeatureRefusal> {
    let Some(name) = common::first_reference_name(ctx.param("neutralPlane")) else {
        return Err(refuse("draft_neutral_plane", "pick a neutral plane: a planar face or a datum plane (the plane that keeps its size, or the parting plane)".into()));
    };
    match common::resolve_plane_reference(ctx, &name) {
        Some(common::PlaneLikeRef::Frame(frame)) => Ok((frame.origin, frame.z_axis.normalized().map_err(|e| refuse("draft_neutral_plane", e))?, None)),
        Some(common::PlaneLikeRef::Face(face)) => {
            // `face_frame` reads the normal at the corners of the parameter
            // domain, which a closed cylinder passes (its corners share one
            // normal, the middle's is opposite): test the surface itself.
            let flat = crate::with_registered_solid_str(face.handle, |solid| Ok(face_is_flat(solid, face.face_id))).unwrap_or(false);
            let frame = (if flat { common::face_frame(face) } else { Err(String::new()) }).map_err(|_| {
                refuse("draft_neutral_plane", format!("the neutral plane `{name}` is not a planar face; pick a flat face or a datum plane"))
            })?;
            let inward = frame.z_axis.normalized().map_err(|e| refuse("draft_neutral_plane", e))?.scale(-1.0);
            Ok((frame.origin, inward, Some(face)))
        }
        None => {
            result.unresolved.push(name.clone());
            Err(refuse("draft_neutral_plane", format!("the neutral plane `{name}` resolves to nothing on this model")))
        }
    }
}

/// Whether face `face_id` of `solid` is flat: its normal the same, sense and
/// all, over a 5 × 5 grid of its parameter domain (a sample with no normal,
/// at a collapsed corner, is passed over).
fn face_is_flat(solid: &BrepSolid, face_id: u64) -> bool {
    let Some(face) = solid.shells.iter().flat_map(|s| s.faces.iter()).find(|f| f.id == face_id) else {
        return false;
    };
    let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else {
        return false;
    };
    let Ok(centre) = face.surface.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)) else {
        return false;
    };
    (0..5).all(|i| {
        (0..5).all(|j| {
            let (u, v) = (u0 + (u1 - u0) * i as f64 / 4.0, v0 + (v1 - v0) * j as f64 / 4.0);
            face.surface.normal(u, v).map_or(true, |normal| normal.dot(centre) >= 1.0 - 1e-9)
        })
    })
}

/// Every face id of `solid`.
fn face_ids(solid: &BrepSolid) -> Vec<u64> {
    solid.shells.iter().flat_map(|shell| shell.faces.iter().map(|face| face.id)).collect()
}

/// The faces standing on the neutral face's boundary: those sharing an edge
/// with its OUTER loop (`outer`) or with its holes (`!outer`). The outer loop
/// is the one enclosing the most area in the face's own plane.
fn faces_on_neutral_boundary(solid: &BrepSolid, neutral_face: u64, normal: Vec3, outer: bool) -> Vec<u64> {
    let Some(face) = solid.shells.iter().flat_map(|s| s.faces.iter()).find(|f| f.id == neutral_face) else {
        return Vec::new();
    };
    let edges: HashMap<u64, _> = solid.edges.iter().map(|e| (e.id, e)).collect();
    // The area a loop encloses, projected on the plane square to `normal`.
    let area = |loop_record: &crate::LoopRecord| -> f64 {
        let mut points = Vec::new();
        for coedge in &loop_record.coedges {
            let Some(edge) = edges.get(&coedge.edge_id) else { continue };
            for step in 0..8 {
                let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / 8.0;
                if let Ok(point) = edge.curve.evaluate(t) {
                    points.push(point);
                }
            }
        }
        let mut sum = Vec3::default();
        for i in 0..points.len() {
            sum = sum.add(points[i].cross(points[(i + 1) % points.len()]));
        }
        sum.dot(normal).abs()
    };
    let Some(outer_index) = (0..face.loops.len()).max_by(|&a, &b| area(&face.loops[a]).total_cmp(&area(&face.loops[b]))) else {
        return Vec::new();
    };
    let wanted: HashSet<u64> = face
        .loops
        .iter()
        .enumerate()
        .filter(|(index, _)| (*index == outer_index) == outer)
        .flat_map(|(_, l)| l.coedges.iter().map(|c| c.edge_id))
        .collect();
    let mut out: Vec<u64> = solid
        .shells
        .iter()
        .flat_map(|s| s.faces.iter())
        .filter(|f| f.id != neutral_face && f.loops.iter().flat_map(|l| l.coedges.iter()).any(|c| wanted.contains(&c.edge_id)))
        .map(|f| f.id)
        .collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// The name a solid goes by in the scene.
fn solid_name(ctx: &FeatureContext, handle: u32) -> Option<String> {
    let mut names: Vec<&String> = ctx.scene.solids.iter().filter(|(_, h)| **h == handle).map(|(name, _)| name).collect();
    names.sort();
    names.first().map(|name| (*name).clone())
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    let kind = choice(ctx, "type", TYPE_NEUTRAL);
    let parted = match kind.as_str() {
        TYPE_NEUTRAL => false,
        TYPE_PARTING => true,
        other => return Err(refuse("draft_type", format!("unknown draft type `{other}` (`{TYPE_NEUTRAL}` or `{TYPE_PARTING}`)"))),
    };
    let face_set = choice(ctx, "faceSet", SET_SELECTED);
    if ![SET_SELECTED, SET_ALL, SET_OUTER, SET_INNER].contains(&face_set.as_str()) {
        return Err(refuse("draft_face_set", format!("unknown face set `{face_set}`")));
    }

    // --- The selected faces (contract rule 1: misses are reported) ----------
    let names = common::reference_name_array(ctx.param("faces"));
    let mut resolved: Vec<(u32, u64, String)> = Vec::new();
    for name in &names {
        match ctx.scene.resolve_face(name) {
            Some(face) => resolved.push((face.handle, face.face_id, name.clone())),
            None => result.unresolved.push(name.clone()),
        }
    }
    if face_set == SET_SELECTED && resolved.is_empty() {
        return Err(if names.is_empty() {
            refuse("draft_no_faces", "select the faces to draft (planes along the pull, or cylinders about it)".into())
        } else {
            refuse("unresolved_reference", format!(
                "none of the {} selected face{} resolves on this model ({}); repair the selection and rerun",
                names.len(),
                if names.len() == 1 { "" } else { "s" },
                names.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(", ")
            ))
        });
    }

    // --- Neutral plane and pull ---------------------------------------------
    let (neutral_point, neutral_normal, neutral_face) = neutral_plane(ctx, &mut result)?;
    let mut pull = neutral_normal;
    if let Some(name) = common::first_reference_name(ctx.param("pullDirection")) {
        // A flat face of a body gives its OUTWARD normal (the side it faces),
        // like a datum plane its own; anything else its direction as the
        // shared direction reference reads it (an edge, an axis, a cylinder).
        let flat_face = ctx.scene.resolve_face(&name).filter(|face| {
            crate::with_registered_solid_str(face.handle, |solid| Ok(face_is_flat(solid, face.face_id))).unwrap_or(false)
        });
        let direction = match flat_face {
            Some(face) => Some(common::face_point_normal(face).map_err(|e| refuse("draft_pull_direction", e))?.1),
            None => crate::feature_pipeline::ports::resolve_direction(ctx.scene, &name).map_err(|e| refuse("draft_pull_direction", e))?,
        };
        match direction {
            Some(direction) => {
                let direction = direction.normalized().map_err(|_| refuse("draft_pull_direction", format!("the pull reference `{name}` gives no direction")))?;
                let off = direction.cross(neutral_normal).length().clamp(0.0, 1.0).asin().to_degrees();
                if off > 1e-4 {
                    return Err(KernelRefusal::unsupported(KernelStage::Classify, "draft_pull_oblique", format!(
                        "{OP}: the pull `{name}` is {off:.4}° off square to the neutral plane; a draft about a \
                         neutral plane that is not square to the pull is not built yet (stage 2) — pick a pull \
                         along the plane's normal, or leave the pull empty to use it"
                    )).into());
                }
                pull = direction;
            }
            None => {
                result.unresolved.push(name.clone());
                return Err(refuse("draft_pull_direction", format!("the pull reference `{name}` resolves to nothing on this model")));
            }
        }
    }
    if flag(ctx, "flipPull", false) {
        pull = pull.scale(-1.0);
    }

    // --- The target body ------------------------------------------------------
    let handles: HashSet<u32> = resolved.iter().map(|(handle, _, _)| *handle).collect();
    if handles.len() > 1 {
        return Err(refuse("draft_several_solids", "the selected faces lie on more than one solid; draft one solid at a time".into()));
    }
    let target_handle = match (handles.iter().next(), neutral_face) {
        (Some(handle), _) => *handle,
        (None, Some(face)) => face.handle,
        (None, None) => {
            let solids: HashSet<u32> = ctx.scene.solids.values().copied().collect();
            if solids.len() != 1 {
                return Err(refuse("draft_target", "pick a face of the solid to draft (the model has more than one solid)".into()));
            }
            *solids.iter().next().expect("one solid")
        }
    };
    let target_name = solid_name(ctx, target_handle).ok_or_else(|| refuse("draft_target", "the solid to draft has no name in the scene".into()))?;
    let solid = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;

    // --- The faces to draft -----------------------------------------------------
    let picked: Vec<u64> = resolved.iter().map(|(_, id, _)| *id).collect();
    let mut chosen: Vec<u64> = match face_set.as_str() {
        SET_SELECTED => picked.clone(),
        SET_ALL => face_ids(&solid).into_iter().filter(|&id| crate::draft_face_qualifies(&solid, id, pull)).collect(),
        set => {
            let Some(face) = neutral_face.filter(|face| face.handle == target_handle) else {
                return Err(refuse("draft_face_set_needs_face", format!(
                    "the `{set}` face set reads the faces standing on the neutral FACE's boundary; pick a face of \
                     the drafted body as the neutral plane, or choose `{SET_SELECTED}` or `{SET_ALL}`"
                )));
            };
            faces_on_neutral_boundary(&solid, face.face_id, neutral_normal, set == SET_OUTER)
                .into_iter()
                .filter(|&id| crate::draft_face_qualifies(&solid, id, pull))
                .collect()
        }
    };
    if flag(ctx, "tangentPropagation", true) {
        chosen = crate::draft_tangent_faces(&solid, &chosen, pull);
    }
    let mut seen = HashSet::new();
    chosen.retain(|id| seen.insert(*id));
    if chosen.is_empty() {
        return Err(refuse("draft_no_faces", format!("the `{face_set}` face set holds no face the draft can take on `{target_name}`")));
    }
    result.note_partial_resolution(&names);

    let face_name = |id: u64| -> String {
        solid
            .shells
            .iter()
            .flat_map(|s| s.faces.iter())
            .find(|f| f.id == id)
            .and_then(|f| f.name.clone())
            .unwrap_or_else(|| format!("#{id}"))
    };
    let added: Vec<String> = chosen.iter().filter(|id| !picked.contains(id)).map(|&id| face_name(id)).collect();
    if !added.is_empty() {
        result.notes.push(format!(
            "{}: drafted {} face{} beyond the picks ({})",
            ctx.id,
            added.len(),
            if added.len() == 1 { "" } else { "s" },
            added.join(", ")
        ));
    }

    // --- Draft ------------------------------------------------------------------
    let angle = angle_degrees(ctx, "angle")?;
    let angle2 = if parted && !flag(ctx, "sameBothSides", true) { angle_degrees(ctx, "angle2")? } else { angle };
    let before: HashMap<u64, String> = common::collect_face_names(&solid).into_iter().collect();
    let mut drafted = if parted {
        crate::draft_faces_parted_sided(&solid, &chosen, pull, neutral_point, angle.to_radians(), angle2.to_radians())?
    } else {
        crate::draft_faces(&solid, &chosen, pull, neutral_point, angle.to_radians())?
    };
    let sides: Vec<(Vec3, f64)> = if parted {
        vec![(pull, angle.to_radians()), (pull.scale(-1.0), angle2.to_radians())]
    } else {
        vec![(pull, angle.to_radians())]
    };
    restore_face_names(&solid, &mut drafted, &before, &chosen, &sides);

    let mut persistent = match ctx.persistent {
        serde_json::Value::Object(map) => serde_json::Value::Object(map.clone()),
        _ => serde_json::json!({}),
    };
    persistent["draft"] = serde_json::json!({
        "pull": [pull.x, pull.y, pull.z],
        "neutral": [neutral_point.x, neutral_point.y, neutral_point.z],
        "faceCount": chosen.len(),
        "parted": parted,
    });
    result.persistent_data = Some(persistent);
    result.added.push(common::register_added(drafted, &target_name));
    result.removed.push(target_name);
    Ok(result)
}

/// Give every face of `after` that came out without its input name (the
/// convex route builds a new body; a parting line splits and unites) the name
/// of the FLAT input face it continues: one whose normal, as the draft leaves
/// it, is the output face's own to within a hundredth of a degree. A drafted
/// face's normal is its outward normal `m` turned toward the pull `d` of its
/// side by that side's angle, `m·cos α + d·sin α`; every other face keeps
/// its normal. Faces that kept their names, and curved ones, are left alone; a
/// name is given once, and a face that matches no input stays unnamed rather
/// than take a wrong one.
fn restore_face_names(before: &BrepSolid, after: &mut BrepSolid, names: &HashMap<u64, String>, chosen: &[u64], sides: &[(Vec3, f64)]) {
    let flat_normal = |solid: &BrepSolid, face: &crate::FaceRecord| -> Option<Vec3> {
        if !face_is_flat(solid, face.id) {
            return None;
        }
        let [u0, u1] = face.surface.domain_u().ok()?;
        let [v0, v1] = face.surface.domain_v().ok()?;
        let normal = face.surface.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).ok()?;
        normal.scale(if face.same_sense { 1.0 } else { -1.0 }).normalized().ok()
    };
    let known: HashSet<&String> = names.values().collect();
    let mut taken: HashSet<String> = after
        .shells
        .iter()
        .flat_map(|s| s.faces.iter())
        .filter_map(|f| f.name.clone())
        .filter(|name| known.contains(name))
        .collect();
    // Each named flat input face and the normals the draft can leave it with.
    let mut expected: Vec<(String, Vec<Vec3>)> = Vec::new();
    for face in before.shells.iter().flat_map(|s| s.faces.iter()) {
        let (Some(name), Some(normal)) = (names.get(&face.id), flat_normal(before, face)) else { continue };
        let normals = if chosen.contains(&face.id) {
            sides.iter().filter_map(|(d, a)| normal.scale(a.cos()).add(d.scale(a.sin())).normalized().ok()).collect()
        } else {
            vec![normal]
        };
        expected.push((name.clone(), normals));
    }
    let snapshot = after.clone();
    let bar = (0.01f64).to_radians().cos();
    for face in after.shells.iter_mut().flat_map(|s| s.faces.iter_mut()) {
        if face.name.as_ref().is_some_and(|name| known.contains(name)) {
            continue;
        }
        let Some(normal) = flat_normal(&snapshot, face) else { continue };
        let matches: Vec<&String> = expected
            .iter()
            .filter(|(name, normals)| !taken.contains(name) && normals.iter().any(|n| n.dot(normal) >= bar))
            .map(|(name, _)| name)
            .collect();
        if let [name] = matches.as_slice() {
            taken.insert((*name).clone());
            face.name = Some((*name).clone());
        }
    }
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

/// The form's field visibility: the second side's angle only for a parting
/// line drafted differently on each side.
pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    let parted = values.get("type").and_then(serde_json::Value::as_str) == Some(TYPE_PARTING);
    let same = values.get("sameBothSides").and_then(serde_json::Value::as_bool).unwrap_or(true);
    let mut hidden = Vec::new();
    if !parted {
        hidden.push("sameBothSides");
    }
    if !parted || same {
        hidden.push("angle2");
    }
    hidden
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "DRAFT",
    "shortName": "DRAFT",
    "longName": "Draft",
    "ribbonPath": "Home/Blend/Draft",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the draft feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": ["FACE"],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "label": "Faces to draft",
            "hint": "The faces to tilt: planes that run along the pull, or cylinders about it (a cylinder becomes a cone). Used when the face set is Selected."
        },
        "type": {
            "type": "options",
            "options": [TYPE_NEUTRAL, TYPE_PARTING],
            "default_value": TYPE_NEUTRAL,
            "label": "Type",
            "hint": "Neutral plane: each face turns about the line where it crosses the neutral plane, which keeps its size. Parting line: the body is split at the plane and each side is drafted away from it, as the two halves of a mould pull apart."
        },
        "neutralPlane": {
            "type": "reference_selection",
            "selectionFilter": ["PLANE", "FACE"],
            "multiple": false,
            "default_value": null,
            "label": "Neutral plane",
            "hint": "A flat face or a datum plane: the plane that keeps its size (or, for a parting line, the parting plane). Its normal is the pull unless a pull direction is picked; a face of the body pulls into the body, so the picked face is where the part is largest."
        },
        "pullDirection": {
            "type": "reference_selection",
            "selectionFilter": ["FACE", "EDGE", "PLANE", "DATUM"],
            "multiple": false,
            "default_value": null,
            "label": "Pull direction",
            "hint": "Optional: the direction the part is pulled out of the mould. A flat face gives its outward normal, a plane its own normal, an edge or axis its direction. It must be square to the neutral plane. Empty: the neutral plane's normal."
        },
        "flipPull": {
            "type": "boolean",
            "default_value": false,
            "label": "Flip pull",
            "hint": "Reverse the pull direction. For a neutral plane this leans the faces the other way; for a parting line it swaps which side takes which angle."
        },
        "angle": {
            "type": "number",
            "step": 0.5,
            "default_value": 3,
            "label": "Angle",
            "hint": "Draft angle in degrees (an expression is accepted). Positive leans each face into the material as it goes along the pull (a boss narrows, a hole widens); negative leans it out. Less than 89° either way."
        },
        "sameBothSides": {
            "type": "boolean",
            "default_value": true,
            "label": "Same angle both sides",
            "hint": "Parting line: draft both sides of the parting plane by the same angle. Off: the side along the pull takes Angle and the other side Angle (other side)."
        },
        "angle2": {
            "type": "number",
            "step": 0.5,
            "default_value": 3,
            "label": "Angle (other side)",
            "hint": "Parting line: the draft angle in degrees on the side against the pull."
        },
        "faceSet": {
            "type": "options",
            "options": [SET_SELECTED, SET_ALL, SET_OUTER, SET_INNER],
            "default_value": SET_SELECTED,
            "label": "Face set",
            "hint": "Selected: the faces picked above. All faces along the pull: every face of the body the draft can take. Outer / Inner: the faces standing on the neutral face's outer edge / on its holes (needs a face of the body as the neutral plane). Read again at every rebuild."
        },
        "tangentPropagation": {
            "type": "boolean",
            "default_value": true,
            "label": "Tangent propagation",
            "hint": "Also draft the faces joined smoothly to the chosen ones (the rounds between walls), where the draft can take them. The added faces are listed on the feature."
        }
    }
})
}

