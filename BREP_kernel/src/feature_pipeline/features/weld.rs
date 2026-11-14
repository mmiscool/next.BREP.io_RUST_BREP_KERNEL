//! History-owned fillet welds. Seam geometry is published even without a bead.
//! Assembly references resolve in world space; the bead belongs to this document.
use super::common;
use crate::feature_pipeline::{FeatureContext, FeatureResult, Frame, ScenePoint, SelectionProbe};
use crate::{make_line, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    build(ctx).unwrap_or_else(|error| ctx.fail(format!("weld: {error}")))
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let seam = selected_seam(ctx.scene, ctx.param("seam"))?;
    let curves = common::resolve_path(ctx, &seam)?;
    let curve = curves.first().ok_or("the seam is empty")?;
    let [t0, t1] = curve.domain()?;
    let anchor = curve.evaluate((t0 + t1) * 0.5)?;
    let size = common::optional_number(ctx, "legSize")?.unwrap_or(3.0);
    if !size.is_finite() || size <= 0.0 {
        return Err("leg size must be finite and positive".into());
    }
    let angle = common::optional_number(ctx, "orientation")?
        .unwrap_or(0.0)
        .to_radians();
    if !angle.is_finite() {
        return Err("orientation must be finite".into());
    }
    let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
    let generate = match ctx.param("generateSolid") {
        None => true,
        Some(serde_json::Value::Bool(value)) => *value,
        Some(_) => return Err("Generate weld solid must be a boolean".into()),
    };
    let linear = curves.len() == 1 && curve.degree == 1 && curve.control_points.len() == 2;
    if linear {
        let start = curve.evaluate(t0)?;
        let end = curve.evaluate(t1)?;
        let delta = end.sub(start);
        let length = delta.length();
        let tangent = delta.normalized()?;
        let frame = seam_frame(ctx, &seam, start, tangent)?;
        result.frames.push((format!("{}:Seam:Frame", ctx.id), frame));
        if generate {
            let u = frame
                .x_axis
                .scale(angle.cos())
                .add(frame.y_axis.scale(angle.sin()));
            let v = tangent.cross(u);
            let a = start.add(u.scale(size));
            let b = start.add(v.scale(size));
            let profile = vec![make_line(start, a)?, make_line(a, b)?, make_line(b, start)?];
            let mut solid =
                crate::extrude_profile_brep(&profile, tangent, length).map_err(|e| e.to_string())?;
            for (i, face) in solid
                .shells
                .iter_mut()
                .flat_map(|s| &mut s.faces)
                .enumerate()
            {
                face.name = Some(format!("{}:BeadFace{}", ctx.id, i + 1));
            }
            result.added.push(common::register_added(solid, &ctx.id));
        }
    }
    result.weld_source_seam = Some(seam);
    result.paths.push((format!("{}:Seam", ctx.id), curves));
    result
        .points
        .push((format!("{}:Callout", ctx.id), ScenePoint::model(anchor)));
    Ok(result)
}

/// Resolve the explicit seam independently of selection order. Additional
/// references constrain its owning face, part or placed component.
fn selected_seam(scene: &crate::feature_pipeline::SceneMap, value: Option<&serde_json::Value>) -> Result<String, String> {
    let names = common::unique_reference_names(value);
    let seams: Vec<_> = names.iter().filter(|name| scene.resolve_edge(name).is_some()
        || scene.resolve_path(name).is_some()
        || name.rsplit_once(':').is_some_and(|(owner, geometry)| geometry.len() > 1 && geometry.starts_with('G') && scene.resolve_path(owner).is_some())).collect();
    let seam = match seams.as_slice() {
        [seam] => (*seam).clone(),
        [] => return Err("select one explicit seam edge or sketch alongside the face, part or component references".into()),
        _ => return Err("select exactly one seam edge or sketch; remove additional seam references".into()),
    };
    for name in names.iter().filter(|name| **name != seam) {
        let edge = scene.resolve_edge(&seam).ok_or("face, part and component scope references require a resident seam edge")?;
        let faces = scene.resolve_face_group(name);
        let belongs = if !faces.is_empty() {
            crate::with_registered_solid_str(edge.handle, |solid| {
                Ok(solid.shells.iter().flat_map(|shell| &shell.faces)
                    .filter(|f| faces.iter().any(|face| face.handle == edge.handle && face.face_id == f.id))
                    .flat_map(|f| &f.loops).flat_map(|l| &l.coedges).any(|c| c.edge_id == edge.edge_id))
            })?
        } else if let Some(handle) = scene.resolve_solid(name) {
            handle == edge.handle
        } else if let Some(component) = scene.resolve_component(name) {
            component.solids.iter().any(|solid| scene.resolve_solid(solid) == Some(edge.handle))
        } else {
            return Err(format!("reference '{name}' is not a resident face, part or component"));
        };
        if !belongs { return Err(format!("seam '{seam}' does not belong to selected reference '{name}'")); }
    }
    Ok(seam)
}

fn direction(pose: crate::AffineTransform, vector: Vec3) -> Vec3 {
    let m = pose.elements;
    Vec3::new(m[0]*vector.x + m[1]*vector.y + m[2]*vector.z,
        m[4]*vector.x + m[5]*vector.y + m[6]*vector.z,
        m[8]*vector.x + m[9]*vector.y + m[10]*vector.z)
}

fn posed_frame(frame: Frame, pose: crate::AffineTransform) -> Frame {
    Frame { origin: pose.point(frame.origin), x_axis: direction(pose, frame.x_axis),
        y_axis: direction(pose, frame.y_axis), z_axis: direction(pose, frame.z_axis) }
}

/// Keep the manual orientation in the seam owner's coordinates, so an authored
/// rotation and an in-place solver rotation produce the same bead on rebuild.
fn seam_frame(ctx: &FeatureContext, seam: &str, start: Vec3, tangent: Vec3) -> Result<Frame, String> {
    if let Some(frame) = ctx.scene.frames.get(&format!("{seam}:Frame")) {
        return Ok(Frame { origin: start, ..*frame });
    }
    let mut owner: Option<(usize, crate::AffineTransform)> = None;
    for (id, record) in ctx.scene.components.iter().chain(ctx.scene.occurrences.iter()) {
        let mut consider = |name: &str, pose| {
            if seam.starts_with(&format!("{name}:")) && owner.as_ref().is_none_or(|(len, _)| name.len() > *len) {
                owner = Some((name.len(), pose));
            }
        };
        consider(id, record.transform);
        for (path, local) in &record.nested_poses {
            consider(&format!("{id}:{path}"), crate::feature_pipeline::component::compose(&record.transform, local)?);
        }
    }
    if let Some((_, pose)) = owner {
        let inverse = pose.rigid_inverse()?;
        let local = Frame::from_origin_normal(inverse.point(start), direction(inverse, tangent))?;
        Ok(posed_frame(local, pose))
    } else {
        Frame::from_origin_normal(start, tangent)
    }
}

/// Apply the seam owner's solved rigid motion without creating or discarding handles.
/// Compute every fallible transformation before committing the replacement bodies.
pub(crate) fn pose(result: &mut FeatureResult, delta: crate::AffineTransform) -> Result<(), String> {
    let bodies = result.added.iter().map(|added| {
        crate::with_registered_solid_str(added.handle, |solid| crate::transform_brep(solid, delta, false))
            .map(|body| (added.handle, body))
    }).collect::<Result<Vec<_>, _>>()?;
    let paths = result.paths.iter().map(|(name, curves)| {
        curves.iter().map(|curve| crate::transform_topology::transform_curve(curve, delta))
            .collect::<Result<Vec<_>, _>>().map(|curves| (name.clone(), curves))
    }).collect::<Result<Vec<_>, _>>()?;
    for (handle, body) in bodies { crate::replace_registered_solid(handle, body)?; }
    result.paths = paths;
    for (_, frame) in &mut result.frames { *frame = posed_frame(*frame, delta); }
    for (_, point) in &mut result.points { point.position = delta.point(point.position); }
    Ok(())
}

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.edges + probe.sketches == 1
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD", "shortName": "WELD", "longName": "Fillet weld",
        "ribbonPath": "Home/Welding/Fillet weld", "commandSize": "Large", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["EDGE", "SKETCH", "FACE", "SOLID", "COMPONENT"], "multiple": true, "allowSingleReference": true, "default_value": null,
                "hint": "Select one seam edge or sketch. Faces, parts and components may be included to constrain the seam owner; each must contain the selected edge."},
            "legSize": {"type": "number", "label": "Leg size", "default_value": 3, "hint": "Equal-leg fillet weld size in model units."},
            "generateSolid": {"type": "boolean", "label": "Generate weld solid", "default_value": true,
                "hint": "On by default: build a separate triangular bead on a single straight seam. Other seams remain annotation-only; turning off retains the weld and PMI."},
            "orientation": {"type": "number", "label": "Orientation", "default_value": 0, "hint": "Rotate the bead around the seam in degrees."},
            "process": {"type": "string", "label": "Process / specification", "default_value": "", "hint": "Optional process or specification shown in linked weld callouts."}
        }
    })
}

