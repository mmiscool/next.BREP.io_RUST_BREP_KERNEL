//! DFF — Deform Face.
//!
//! Push or pull one face of a solid by its surface's control net: each pick
//! is a point on the face that moves a given distance along the face's
//! outward normal, optionally keeping its tangent plane (a flat top over the
//! pick). Every edge, vertex and neighbouring face stays exactly as it was:
//! the face's carrier keeps its points and tangent planes along every trim.
//! Underlying kernel fn: [`crate::deform_face`], which refuses by name — an
//! anchor off the face, a push bigger than the face, picks closer than the
//! net can separate or too near a held edge, a fold, a result that passes
//! through the body — and changes nothing when it does.
//!
//! Several faces (Golovanov §6.13): faces of one body that meet along whole
//! sides of their carriers deform together through [`crate::deform_shell`];
//! the edges between them move with them (kept tangent where they were
//! smooth), every other edge, vertex and face stays exactly as it was.
//!
//! Reference resolution (contract rule 1): `face` is a FACE
//! `reference_selection` of one face or several (a single name, as documents
//! saved before several were allowed carry, still reads); a miss is an
//! `unresolved` entry, and nothing resolved is a soft no-op.
//!
//! The picks live in `persistentData.deform.picks`: a list of
//! `{ "anchor": [x, y, z], "offset": number | expression }` (an `offset` left
//! out takes the form's `offset`). Each pick belongs to the selected face it
//! is nearest to. With no list, one pick sits at each face's deepest interior
//! point. Each offset is evaluated as an expression against
//! the shared environment, like any numeric param. The feature's result
//! carries the resolved picks back in its own `persistentData.deform`
//! (anchor, outward normal, offset, and the point it was moved to) for the
//! viewport handles.
//!
//! Name fidelity (contract rule 2): the result reuses the target solid's name
//! and every face and edge keeps its name (the topology does not change).

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureRefusal, FeatureResult};
use crate::{
    deform_face, deform_shell, face_deform_default_anchor, FaceDeformPick, FaceDeformSpec, ShellDeformPick, ShellDeformSpec,
    Vec3,
};

pub const DEFAULT_OFFSET: f64 = 2.0;
pub const DEFAULT_RESOLUTION: f64 = 8.0;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn number_or(ctx: &FeatureContext, key: &str, default: f64) -> Result<f64, FeatureRefusal> {
    match ctx.param(key) {
        None | Some(serde_json::Value::Null) => Ok(default),
        Some(serde_json::Value::String(text)) if text.trim().is_empty() => Ok(default),
        Some(_) => Ok(ctx.number(key)?),
    }
}

fn vec3_of(value: &serde_json::Value) -> Option<Vec3> {
    let array = value.as_array()?;
    if array.len() != 3 {
        return None;
    }
    let c: Vec<f64> = array.iter().filter_map(serde_json::Value::as_f64).collect();
    (c.len() == 3 && c.iter().all(|x| x.is_finite())).then(|| Vec3::new(c[0], c[1], c[2]))
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    // One face (a single name, as documents saved before several faces were
    // allowed carry) or several.
    let names = common::reference_names(ctx.param("face"));
    let mut resolved_faces = Vec::new();
    for name in &names {
        match ctx.scene.resolve_face(name) {
            Some(face_ref) => resolved_faces.push(face_ref),
            None => result.unresolved.push(name.clone()),
        }
    }
    let Some(first) = resolved_faces.first().copied() else {
        return Ok(result); // Soft no-op — nothing selected (or nothing resolved).
    };
    if resolved_faces.iter().any(|face_ref| face_ref.handle != first.handle) {
        return Err("Deform Face: the selected faces belong to more than one solid; pick faces of one body".into());
    }
    result.note_partial_resolution(&names);
    let offset = number_or(ctx, "offset", DEFAULT_OFFSET)?;
    let resolution = number_or(ctx, "resolution", DEFAULT_RESOLUTION)?;
    if !(resolution.is_finite() && (1.0..=64.0).contains(&resolution)) {
        return Err(format!("Deform Face: resolution {resolution} must be between 1 and 64").into());
    }
    let keep_tangent = ctx.param("keepTangent").and_then(serde_json::Value::as_bool).unwrap_or(true);

    let target_name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, handle)| **handle == first.handle)
        .map(|(name, _)| name.clone())
        .ok_or("deform_face: target solid has no scene-map name")?;
    let solid = crate::with_registered_solid_str(first.handle, |solid| Ok(solid.clone()))?;
    let mut faces = Vec::new();
    for face_ref in &resolved_faces {
        if faces.iter().any(|f: &crate::topology::FaceRecord| f.id == face_ref.face_id) {
            continue;
        }
        let face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_ref.face_id)
            .ok_or("deform_face: a selected face is not in its solid")?
            .clone();
        faces.push(face);
    }

    // The picks: the persisted list, or one at each face's deepest point.
    let mut anchors: Vec<(Vec3, f64)> = Vec::new();
    if let Some(list) = ctx.persistent.pointer("/deform/picks").and_then(serde_json::Value::as_array) {
        for (index, entry) in list.iter().enumerate() {
            let anchor = entry
                .get("anchor")
                .and_then(vec3_of)
                .ok_or_else(|| format!("Deform Face: pick {index} has no anchor [x, y, z]"))?;
            let value = match entry.get("offset") {
                None | Some(serde_json::Value::Null) => offset,
                Some(serde_json::Value::Number(number)) => number.as_f64().unwrap_or(offset),
                Some(serde_json::Value::String(text)) if text.trim().is_empty() => offset,
                Some(serde_json::Value::String(text)) => ctx
                    .env
                    .eval(text)
                    .map_err(|error| format!("Deform Face: pick {index} offset: {error}"))?,
                Some(other) => return Err(format!("Deform Face: pick {index} offset must be a number or expression, found {other}").into()),
            };
            anchors.push((anchor, value));
        }
    }
    if anchors.is_empty() {
        for face in &faces {
            let anchor = face_deform_default_anchor(face).ok_or("Deform Face: a face has no interior point to push")?;
            anchors.push((anchor, offset));
        }
    }

    // Each pick on the face it is nearest to, moved along that face's
    // OUTWARD normal at its foot.
    let mut picks: Vec<(usize, FaceDeformPick)> = Vec::with_capacity(anchors.len());
    let mut resolved = Vec::with_capacity(anchors.len());
    for (index, &(anchor, value)) in anchors.iter().enumerate() {
        if !value.is_finite() {
            return Err(format!("Deform Face: pick {index} offset is not finite").into());
        }
        let mut best: Option<(usize, crate::SurfaceProjection)> = None;
        for (f, face) in faces.iter().enumerate() {
            if let Ok(foot) = crate::project_point_to_surface(&face.surface, anchor) {
                if best.as_ref().is_none_or(|(_, b)| foot.distance < b.distance) {
                    best = Some((f, foot));
                }
            }
        }
        let (f, foot) = best.ok_or_else(|| format!("Deform Face: pick {index} could not be placed on a face"))?;
        let face = &faces[f];
        let on = face.surface.evaluate(foot.u, foot.v)?;
        let mut normal = face.surface.normal(foot.u, foot.v)?;
        if !face.same_sense {
            normal = normal.scale(-1.0);
        }
        let target = on.add(normal.scale(value));
        // The anchor goes to the kernel as given: it projects it again and
        // refuses one that is off the face, rather than this snapping it on.
        picks.push((f, FaceDeformPick { anchor, target, normal: keep_tangent.then_some(normal) }));
        resolved.push(serde_json::json!({
            "anchor": [on.x, on.y, on.z],
            "normal": [normal.x, normal.y, normal.z],
            "offset": value,
            "target": [target.x, target.y, target.z],
        }));
    }
    let resolution = resolution.round() as usize;
    let solid = if faces.len() == 1 {
        let spec = FaceDeformSpec { picks: picks.into_iter().map(|(_, p)| p).collect(), resolution, ..FaceDeformSpec::default() };
        let (solid, report) = deform_face(&solid, faces[0].id, &spec)?;
        result.notes.push(format!(
            "deformed face on a {}x{} net ({} control points free), largest pick error {:.1e}",
            report.net.0, report.net.1, report.free_poles, report.max_pick_error
        ));
        solid
    } else {
        let spec = ShellDeformSpec {
            picks: picks
                .into_iter()
                .map(|(f, p)| ShellDeformPick { face_id: faces[f].id, anchor: p.anchor, target: p.target, normal: p.normal })
                .collect(),
            resolution,
            ..ShellDeformSpec::default()
        };
        let ids: Vec<u64> = faces.iter().map(|face| face.id).collect();
        let (solid, report) = deform_shell(&solid, &ids, &spec)?;
        let tangent = report.shared_edges.iter().filter(|(_, c)| matches!(c, crate::TieContinuity::Tangent { .. })).count();
        result.notes.push(format!(
            "deformed {} faces together; {} shared edges ({} kept tangent, {} sharp), largest pick error {:.1e}",
            faces.len(),
            report.shared_edges.len(),
            tangent,
            report.shared_edges.len() - tangent,
            report.max_pick_error
        ));
        solid
    };
    // The result's persistentData replaces the feature's: keep everything the
    // feature was given (its picks and their expressions) and add `resolved`.
    let mut persistent = match ctx.persistent {
        serde_json::Value::Object(map) => serde_json::Value::Object(map.clone()),
        _ => serde_json::json!({}),
    };
    if !persistent.get("deform").is_some_and(serde_json::Value::is_object) {
        persistent["deform"] = serde_json::json!({});
    }
    persistent["deform"]["resolved"] = serde_json::Value::Array(resolved);
    result.persistent_data = Some(persistent);

    let face_names = common::collect_face_names(&solid);
    let edge_names = common::collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    result.added.push(AddedSolid { handle, name: target_name.clone(), face_names, edge_names, ..AddedSolid::default() });
    result.removed.push(target_name);
    Ok(result)
}

/// Context-bar applicability: one or more selected faces drive `face`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces >= 1
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "DFF",
    "shortName": "DFF",
    "longName": "Deform Face",
    "ribbonPath": "Home/Modeling/Deform Face",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the deform face feature"
        },
        "face": {
            "type": "reference_selection",
            "selectionFilter": ["FACE"],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "The face to push or pull, or several faces that meet along whole sides (they deform together and the edges between them move with them, kept tangent where they were smooth). The outer edges and every other face stay exactly where they are"
        },
        "offset": {
            "type": "number",
            "default_value": DEFAULT_OFFSET,
            "hint": "How far the pick points move along the face's outward normal (negative pulls inward). Drag a pick's handle in the viewport to change it; a pick may carry its own offset"
        },
        "keepTangent": {
            "type": "boolean",
            "default_value": true,
            "hint": "Keep the face's tangent plane at each pick, so the pushed region is flat on top over the pick"
        },
        "resolution": {
            "type": "number",
            "default_value": DEFAULT_RESOLUTION,
            "min": 1,
            "step": 1,
            "hint": "How finely the face's surface is divided before it is deformed (1 to 64). Higher spreads the bump less and lets picks sit closer to the edges and to each other"
        }
    }
})
}

