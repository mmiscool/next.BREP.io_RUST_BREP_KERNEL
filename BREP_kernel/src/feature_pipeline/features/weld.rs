//! History-owned fillet welds. Seam geometry is published even without a bead.
//! Assembly references resolve in world space; the bead belongs to this document.
use super::common;
use crate::feature_pipeline::{FeatureContext, FeatureResult, Frame, ScenePoint, SelectionProbe};
use crate::Vec3;

#[path = "weld_fit.rs"]
mod fit;

/// How the bead section is placed around a straight seam.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Placement {
    /// Fit the joint faces; use the manual orientation where none fit.
    Auto,
    /// Fit the joint faces or fail by name.
    Fitted,
    /// Orientation only, as authored.
    Manual,
}

/// The Placement dropdown; AUTO when the key is absent (its schema default).
fn placement(ctx: &FeatureContext) -> Result<Placement, String> {
    match ctx.param("placement") {
        Some(serde_json::Value::String(value)) => match value.as_str() {
            "AUTO" => Ok(Placement::Auto),
            "FITTED" => Ok(Placement::Fitted),
            "MANUAL" => Ok(Placement::Manual),
            other => Err(format!("unknown placement '{other}' (AUTO, FITTED or MANUAL)")),
        },
        None => Ok(Placement::Auto),
        Some(_) => Err("placement must be AUTO, FITTED or MANUAL".into()),
    }
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    build(ctx).unwrap_or_else(|error| ctx.fail(format!("weld: {error}")))
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let (seams, joint_faces) = selected_seam(ctx.scene, ctx.param("seam"))?;
    let seam = seams[0].clone();
    let curves: Vec<crate::NurbsCurve> = common::resolve_path_chain(ctx, &seams)?.into_iter().map(|segment| segment.curve).collect();
    let curve = curves.first().ok_or("the seam is empty")?;
    let [t0, t1] = curve.domain()?;
    let anchor = curve.evaluate((t0 + t1) * 0.5)?;
    let size = common::optional_number(ctx, "legSize")?.unwrap_or(3.0);
    if !size.is_finite() || size <= 0.0 {
        return Err("leg size must be finite and positive".into());
    }
    let second = common::optional_number(ctx, "legSize2")?.unwrap_or(0.0);
    if !second.is_finite() || second < 0.0 {
        return Err("second leg size must be finite and not negative (0 = equal legs)".into());
    }
    let second = if second == 0.0 { size } else { second };
    let swap = match ctx.param("swapLegs") {
        None => false,
        Some(serde_json::Value::Bool(value)) => *value,
        Some(_) => return Err("Swap legs must be a boolean".into()),
    };
    let legs = if swap { [second, size] } else { [size, second] };
    let placement = placement(ctx)?;
    let colour = match ctx.param("weldColor") {
        None => WELD_COLOUR.to_string(),
        Some(serde_json::Value::String(hex)) if is_hex_colour(hex) => hex.to_ascii_lowercase(),
        Some(_) => return Err("weld colour must be `#rrggbb`".into()),
    };
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
    let owner = ctx.scene.resolve_edge(&seam).and_then(|edge| {
        ctx.scene.solids.iter().filter(|(_, handle)| **handle == edge.handle)
            .map(|(name, _)| name.clone()).min()
    });
    // The seam's runs: straight lines and circular arcs, head to tail.
    let mut runs = Vec::new();
    let mut shape: Option<String> = None;
    for (index, curve) in curves.iter().enumerate() {
        match fit::Seam::from_curve(curve, owner.clone())? {
            Some(run) => runs.push(run),
            None => {
                shape = Some(if curves.len() == 1 {
                    "the seam is neither straight nor a circular arc".into()
                } else {
                    format!("segment {} of the seam is neither straight nor a circular arc", index + 1)
                });
                break;
            }
        }
    }
    let chain = match shape {
        Some(reason) => Err(reason),
        None => fit::Chain::new(runs),
    };
    let chain = match chain {
        Ok(chain) => chain,
        Err(reason) => {
            faces_hold_seam_edge(ctx.scene, &seam, &joint_faces)?;
            if generate {
                not_built(&mut result, &seam, format!("has no weld bead: {reason}; the weld and its callout stand"));
            }
            return Ok(finish(ctx, result, seam, curves, anchor));
        }
    };
    let first = &chain.runs[0];
    let frame = seam_frame(ctx, &seam, first.start, first.tangent)?;
    result.frames.push((format!("{}:Seam:Frame", ctx.id), frame));
    // A picked face carries some run of the seam, stands a bridgeable root
    // gap off a single straight seam, or holds the seam edge.
    for name in &joint_faces {
        let mut carried = false;
        for run in &chain.runs {
            carried = carried || fit::carries(ctx.scene, run, name)?;
        }
        if !carried && chain.is_single_line() {
            carried = fit::bridges(ctx.scene, first, name, legs)?;
        }
        if !carried {
            faces_hold_seam_edge(ctx.scene, &seam, std::slice::from_ref(name))?;
        }
    }
    let reach = legs[0].max(legs[1]) + 100.0 * crate::tolerance::WELD_FLOOR;
    let fitting = placement != Placement::Manual && generate || !joint_faces.is_empty();
    let fitted = if fitting {
        match chain.trims(reach) {
            Ok(trims) => chain.fit(ctx.scene, &joint_faces, &ctx.id, legs)?.map(|fits| (fits, trims)),
            Err(reason) => Err(reason),
        }
    } else {
        Err("manual placement".into())
    };
    let mut unfitted: Option<String> = None;
    let fitted = match fitted {
        Ok(joint) if placement != Placement::Manual => Some(joint),
        Err(reason) if placement == Placement::Fitted && generate => {
            return Err(format!("fitted placement: {reason}"));
        }
        Err(reason) if placement == Placement::Auto => {
            unfitted = Some(reason);
            None
        }
        _ => None,
    };
    if generate {
        // MANUAL builds at the authored Orientation: about the seam frame on a
        // single straight seam, else from the plane the seam lies in (0 lays
        // leg 1 across the seam in that plane, leg 2 along its normal). An AUTO
        // weld with no corner turns the bead in quarter turns from it until the
        // bead sits outside every body, so it is never hidden inside the metal.
        let plane = chain.plane();
        let manual = |turn: f64| -> Result<Result<(crate::BrepSolid, Vec3), String>, String> {
            let a = angle + turn;
            let sections: Vec<[Vec3; 3]> = if chain.is_single_line() {
                let u = frame.x_axis.scale(a.cos()).add(frame.y_axis.scale(a.sin()));
                vec![[Vec3::ZERO, u.scale(legs[0]), first.tangent.cross(u).scale(legs[1])]]
            } else {
                let Some(up) = plane else {
                    return Ok(Err("the seam does not lie in one plane, so Orientation cannot place a bead".into()));
                };
                chain.runs.iter().map(|run| {
                    let side = up.cross(run.tangent);
                    let u = side.scale(a.cos()).add(up.scale(a.sin()));
                    let v = up.scale(a.cos()).sub(side.scale(a.sin()));
                    [Vec3::ZERO, u.scale(legs[0]), v.scale(legs[1])]
                }).collect()
            };
            let trims = match chain.trims(reach) {
                Ok(trims) => trims,
                Err(reason) => return Ok(Err(reason)),
            };
            // The section triangle's centroid, halfway along the first run.
            let [_, toe_u, toe_v] = sections[0];
            let centroid = first.at(0.5).add(first.carry(0.5, toe_u.add(toe_v).scale(1.0 / 3.0)));
            Ok(Ok((chain.bead(&ctx.id, &sections, &trims)?, centroid)))
        };
        let auto_manual = |reason: &str, result: &mut FeatureResult| -> Result<Option<crate::BrepSolid>, String> {
            // A turn whose bead cannot be built (a section across an arc's axis)
            // is skipped; the next turn may build.
            let mut first_built: Option<crate::BrepSolid> = None;
            let mut failure: Option<String> = None;
            for quarter in 0..4 {
                let (solid, centroid) = match manual(quarter as f64 * std::f64::consts::FRAC_PI_2) {
                    Ok(Ok(built)) => built,
                    Ok(Err(why)) => {
                        not_built(result, &seam, format!("has no weld bead: {reason}; {why}"));
                        return Ok(None);
                    }
                    Err(error) => {
                        failure.get_or_insert(error);
                        continue;
                    }
                };
                if outside_bodies(ctx.scene, centroid, &ctx.id)? {
                    not_built(result, &seam, format!("has an unfitted bead: {reason}; the bead is placed by Orientation{}",
                        if quarter == 0 { String::new() } else { format!(", turned {} degrees to sit outside the parts", quarter * 90) }));
                    return Ok(Some(solid));
                }
                first_built.get_or_insert(solid);
            }
            match first_built {
                Some(solid) => {
                    not_built(result, &seam, format!("has an unfitted bead: {reason}; no Orientation keeps the bead outside the parts"));
                    Ok(Some(solid))
                }
                None => {
                    not_built(result, &seam, format!("has no weld bead: {reason}; no Orientation builds one ({})", failure.unwrap_or_default()));
                    Ok(None)
                }
            }
        };
        let solid = match &fitted {
            Some((fits, trims)) => {
                let sections: Vec<[Vec3; 3]> = fits.iter().map(|joint| [joint.root, joint.legs[0].scale(legs[0]), joint.legs[1].scale(legs[1])]).collect();
                let solid = chain.bead(&ctx.id, &sections, trims)?;
                match chain.verify(ctx.scene, fits, legs, &solid, trims, &ctx.id)? {
                    Ok(()) => {
                        // Leg order, each body once.
                        for solid in fits.iter().flat_map(|joint| joint.faces.iter().map(|face| face.solid.clone())) {
                            if !result.weld_joint_solids.contains(&solid) {
                                result.weld_joint_solids.push(solid);
                            }
                        }
                        Some(solid)
                    }
                    Err(reason) if placement == Placement::Fitted => return Err(format!("fitted placement: {reason}")),
                    Err(reason) => auto_manual(&reason, &mut result)?,
                }
            }
            None => match &unfitted {
                Some(reason) => auto_manual(reason, &mut result)?,
                None => match manual(0.0)? {
                    Ok((solid, _)) => Some(solid),
                    Err(why) => {
                        not_built(&mut result, &seam, format!("has no weld bead: {why}"));
                        None
                    }
                },
            },
        };
        if let Some(solid) = solid {
            result.added.push(common::register_added(solid, &ctx.id));
            paint_bead(&ctx.id, &colour);
        }
    }
    Ok(finish(ctx, result, seam, curves, anchor))
}

/// Publish the seam path, its source name and the callout point.
fn finish(ctx: &FeatureContext, mut result: FeatureResult, seam: String, curves: Vec<crate::NurbsCurve>, anchor: Vec3) -> FeatureResult {
    result.weld_source_seam = Some(seam);
    result.paths.push((format!("{}:Seam", ctx.id), curves));
    result
        .points
        .push((format!("{}:Callout", ctx.id), ScenePoint::model(anchor)));
    result
}

/// The bead colour a new weld shows: a warm bronze that stands out against the
/// default face colour, so welds are easy to find in the model.
pub const WELD_COLOUR: &str = "#d9822b";

fn is_hex_colour(text: &str) -> bool {
    text.len() == 7 && text.starts_with('#') && text[1..].bytes().all(|b| b.is_ascii_hexdigit())
}

/// Give the bead solid the weld colour as its metadata `color`, the colour the
/// 3D view, the Metadata tab and the exporters read. `weldColor` records what
/// the weld last applied: a colour the user set on the bead in the Metadata
/// tab differs from it and is kept.
fn paint_bead(name: &str, colour: &str) {
    let record = crate::feature_pipeline::scene_metadata::own_record(name).unwrap_or_default();
    let current = record.get("color").and_then(serde_json::Value::as_str);
    let applied = record.get("weldColor").and_then(serde_json::Value::as_str);
    if current.is_none() || current == applied {
        let mut patch = serde_json::Map::new();
        patch.insert("color".into(), serde_json::Value::from(colour));
        patch.insert("weldColor".into(), serde_json::Value::from(colour));
        crate::feature_pipeline::scene_metadata::merge_record(name, &patch, true);
    }
}

/// Report what the weld could not build on its seam, beside the feature in the
/// history (the weld itself stands).
fn not_built(result: &mut FeatureResult, seam: &str, reason: String) {
    result.fulfilment = Some(crate::feature_pipeline::Fulfilment {
        requested: vec![seam.to_string()],
        applied: vec![seam.to_string()],
        rejected: vec![crate::feature_pipeline::Rejected {
            name: seam.to_string(),
            kind: crate::feature_pipeline::RejectedKind::NotBuilt,
            reason,
        }],
        ..Default::default()
    });
}

/// Whether the bead's centroid lies outside every resident body (beads of
/// other welds included).
fn outside_bodies(scene: &crate::feature_pipeline::SceneMap, centroid: Vec3, exclude: &str) -> Result<bool, String> {
    for (name, handle) in &scene.solids {
        if name == exclude { continue; }
        let inside = crate::with_registered_solid_str(*handle, |body| {
            Ok(crate::classify_point(centroid, body, 1e-7)?.class != crate::PointClass::Out)
        })?;
        if inside { return Ok(false); }
    }
    Ok(true)
}

/// Resolve the explicit seam independently of selection order: one or more
/// seam edges or sketch paths that join head to tail (the first picked names
/// the seam). Faces are the joint faces (in selection order); parts and
/// components constrain the seam owner and must contain every seam edge.
fn selected_seam(scene: &crate::feature_pipeline::SceneMap, value: Option<&serde_json::Value>) -> Result<(Vec<String>, Vec<String>), String> {
    let names = common::unique_reference_names(value);
    let seams: Vec<String> = names.iter().filter(|name| scene.resolve_edge(name).is_some()
        || scene.resolve_path(name).is_some()
        || name.rsplit_once(':').is_some_and(|(owner, geometry)| geometry.len() > 1 && geometry.starts_with('G') && scene.resolve_path(owner).is_some()))
        .cloned().collect();
    if seams.is_empty() {
        // Two joint faces alone: the seam is the straight or circular edge they meet along.
        if names.len() == 2 && names.iter().all(|name| !scene.resolve_face_group(name).is_empty()) {
            let seam = fit::seam_between(scene, [&names[0], &names[1]])??;
            return Ok((vec![seam], names.clone()));
        }
        return Err("select the seam edges or sketch, or the two joint faces, alongside the part or component references".into());
    }
    let mut faces = Vec::new();
    for name in names.iter().filter(|name| !seams.contains(name)) {
        if !scene.resolve_face_group(name).is_empty() {
            faces.push(name.clone());
            continue;
        }
        for seam in &seams {
            let edge = scene.resolve_edge(seam).ok_or("part and component scope references require resident seam edges")?;
            let belongs = if let Some(handle) = scene.resolve_solid(name) {
                handle == edge.handle
            } else if let Some(component) = scene.resolve_component(name) {
                component.solids.iter().any(|solid| scene.resolve_solid(solid) == Some(edge.handle))
            } else {
                return Err(format!("reference '{name}' is not a resident face, part or component"));
            };
            if !belongs { return Err(format!("seam '{seam}' does not belong to selected reference '{name}'")); }
        }
    }
    Ok((seams, faces))
}

/// The v1 rule for a face picked beside a seam that cannot be fitted (curved
/// or multi-segment): the face must hold the seam edge.
fn faces_hold_seam_edge(scene: &crate::feature_pipeline::SceneMap, seam: &str, faces: &[String]) -> Result<(), String> {
    for name in faces {
        let edge = scene.resolve_edge(seam).ok_or("face references on a curved or multi-segment seam require a resident seam edge")?;
        let group = scene.resolve_face_group(name);
        let belongs = crate::with_registered_solid_str(edge.handle, |solid| {
            Ok(solid.shells.iter().flat_map(|shell| &shell.faces)
                .filter(|f| group.iter().any(|face| face.handle == edge.handle && face.face_id == f.id))
                .flat_map(|f| &f.loops).flat_map(|l| &l.coedges).any(|c| c.edge_id == edge.edge_id))
        })?;
        if !belongs { return Err(format!("seam '{seam}' does not belong to selected reference '{name}'")); }
    }
    Ok(())
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

/// Whether a weld descriptor may fit its bead to faces it does not name: the
/// history cache must then treat every resident body as an input.
pub(crate) fn fits_joint_faces(descriptor: &crate::feature_pipeline::FeatureDescriptor) -> bool {
    matches!(descriptor.feature_type.as_str(), "WELD")
        && descriptor.input_params.get("placement").and_then(|value| value.as_str()) != Some("MANUAL")
        && descriptor.input_params.get("generateSolid") != Some(&serde_json::Value::Bool(false))
}

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.edges + probe.sketches >= 1
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type": "WELD", "shortName": "WELD", "longName": "Fillet weld",
        "ribbonPath": "Home/Welding/Fillet weld", "commandSize": "Large", "displayBuilder": true,
        "inputParamsSchema": {
            "id": {"type": "string", "default_value": null},
            "seam": {"type": "reference_selection", "label": "Weld references", "selectionFilter": ["EDGE", "SKETCH", "FACE", "SOLID", "COMPONENT"], "multiple": true, "default_value": null,
                "hint": "Select the seam: one or more edges or sketch segments joined end to end. Add the joint faces to choose the corner (the first takes Leg size); parts and components constrain the seam owner and must contain the edges."},
            "legSize": {"type": "number", "label": "Leg size", "default_value": 3, "hint": "Fillet weld leg along the first joint face, in model units."},
            "legSize2": {"type": "number", "label": "Second leg", "default_value": 0, "hint": "Leg along the second joint face. 0 = equal legs."},
            "swapLegs": {"type": "boolean", "label": "Swap legs", "default_value": false, "hint": "Exchange which joint face each leg runs along."},
            "placement": {"type": "options", "label": "Placement", "options": ["AUTO", "FITTED", "MANUAL"], "default_value": "AUTO",
                "hint": "AUTO fits the bead into the corner of the two faces that meet along the seam and uses Orientation where no corner is found. FITTED requires the corner. MANUAL uses Orientation only."},
            "generateSolid": {"type": "boolean", "label": "Generate weld solid", "default_value": true,
                "hint": "On by default: build a separate triangular bead along the seam's straight and circular segments, fitted to the joint faces and mitred at corners. Other seams remain annotation-only; turning off retains the weld and PMI."},
            "orientation": {"type": "number", "label": "Orientation", "default_value": 0, "hint": "Manual placement: rotate the bead around the seam in degrees (on a curved or cornered seam, from across the seam in the seam's plane)."},
            "weldColor": {"type": "color", "label": "Weld colour", "default_value": "#d9822b",
                "hint": "Colour of the bead solid in the 3D view and exports. A colour set on the bead in the Metadata tab is kept."},
            "process": {"type": "string", "label": "Process / specification", "default_value": "", "hint": "Optional process or specification shown in linked weld callouts."}
        }
    })
}

