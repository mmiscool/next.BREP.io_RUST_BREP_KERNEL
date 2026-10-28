//! BOM find-number callout, resolved live from the owning occurrence.
//!
//! The arrow head lands ON the occurrence as the owning view DISPLAYS it: by
//! default at the point of its visible, posed, un-sectioned surface nearest
//! the bubble in screen space, or — with the optional `anchor` reference — on a picked edge
//! (nearest the bubble) or exactly at a picked vertex. A target the view
//! cannot show (every member hidden, or sectioned away; an anchor on a
//! hidden member or on the cut-away side) is an error row with no leader,
//! never a leader into space. See [`crate::feature_pipeline::pmi::anchor`]
//! for the projection, the per-solid explode stages, the section half-space
//! and the recipe the viewport re-derives the head with while the bubble is
//! dragged.
use std::collections::BTreeMap;

use super::{id_field, params, reference_field, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::anchor::{
    nearest_screen_point, head_distance, nearest_displayed_point_on_edge, pose_apply, pose_unapply, BalloonAnchor,
    BalloonAnchorMode, HalfSpace, HeadScope, PmiPose, SolidPoses,
};
use crate::feature_pipeline::pmi::resolve::{a3, solid_bbox, v3};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiContext, PmiGeometry, PmiView, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::{resolve_vertex_selection, SelectionGeometry, Vec3};

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "balloon",
    short_name: "BAL",
    icon: "\u{2460}",
    long_name: "\u{2460} Find-number balloon",
    label: "Find-number balloon",
    applicable: |p: &SelectionProbe| p.components > 0,
    schema,
    resolve,
};
fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            (
                "target",
                reference_field(
                    "Component or geometry",
                    &["COMPONENT", "SOLID", "FACE", "EDGE", "VERTEX"],
                    false,
                    1,
                    1,
                    "The occurrence whose BOM Find Number is displayed",
                ),
            ),
            (
                "anchor",
                reference_field(
                    "Arrow anchor",
                    &["VERTEX", "EDGE"],
                    false,
                    0,
                    1,
                    "Optional: a vertex or an edge of the component the arrow head lands on; empty puts it on the nearest point of the surface in screen space",
                ),
            ),
            (
                "occurrence",
                string_field(
                    "Occurrence",
                    "",
                    "Optional: a placement inside a sub-assembly (ACOMP1:ACOMP3) to balloon instead of the top-level component; its Find Number is the one its sub-assembly gives it",
                ),
            ),
        ]),
    )
}

/// The view the annotation belongs to, from the PMI block being resolved.
fn owning_view<'a>(annotation: &PmiAnnotation, context: &'a PmiContext<'_>) -> Option<&'a PmiView> {
    context.state.find_annotation(annotation.id()).map(|(view, _)| view)
}

/// The explode stages the owning view displays each of `solids` through:
/// every enabled, resolved explode row naming the solid, in the view's row
/// order — the order the viewport poses the display in.
fn explode_stages(view: Option<&PmiView>, context: &PmiContext<'_>, solids: &[String]) -> BTreeMap<String, Vec<PmiPose>> {
    let mut stages: BTreeMap<String, Vec<PmiPose>> = BTreeMap::new();
    let Some(view) = view else {
        return stages;
    };
    for explode in view
        .annotations
        .iter()
        .filter(|a| a.enabled && a.kind == "explode")
    {
        let Ok(resolved) = (super::explode::DEF.resolve)(explode, context) else { continue };
        if let PmiGeometry::Explode {
            solids: exploded,
            center,
            translate,
            rotate_deg,
            scale,
            ..
        } = resolved.geometry
        {
            let pose = PmiPose {
                center,
                translate,
                rotate_deg,
                scale,
            };
            for name in solids.iter().filter(|name| exploded.contains(name)) {
                stages.entry(name.clone()).or_default().push(pose);
            }
        }
    }
    stages
}

/// The view's section half-space (all its planes), kept where the display
/// clip keeps.
fn section_of(view: Option<&PmiView>) -> Option<HalfSpace> {
    HalfSpace::of(&view?.section_planes())
}

/// Parse a `{solid}@x,y,z` vertex reference.
fn parse_vertex_ref(name: &str) -> Option<(&str, Vec3)> {
    let (solid, coords) = name.split_once('@')?;
    let mut parts = coords.split(',').map(|part| part.trim().parse::<f64>());
    let x = parts.next()?.ok()?;
    let y = parts.next()?.ok()?;
    let z = parts.next()?.ok()?;
    if parts.next().is_some() {
        return None;
    }
    Some((solid, Vec3::new(x, y, z)))
}

/// The occurrence a balloon names, with what the owning view makes of it.
struct Occurrence {
    id: String,
    /// Every member `(scene name, handle)`.
    members: Vec<(String, u32)>,
    /// The members the view shows (not in its hidden list).
    visible: Vec<(String, u32)>,
    stages: BTreeMap<String, Vec<PmiPose>>,
    section: Option<HalfSpace>,
    /// Members the view's section leaves whole: never sectioned away.
    exempt: Vec<String>,
    /// The bubble a balloon without a stored one gets: off the displayed
    /// box's +X+Y corner at mid height, outside the body.
    default_label: [f64; 3],
}

fn occurrence(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Occurrence, String> {
    let targets = annotation.references("target");
    if targets.len() != 1 {
        return Err("select exactly one component or element".into());
    }
    let target = targets
        .first()
        .ok_or("select a component or its geometry")?;
    let component = context
        .scene
        .owning_component(target)
        .ok_or("select geometry belonging to an assembly component")?;
    let (id, members) = match nested_path(annotation) {
        Some(path) => {
            if !path.starts_with(&format!("{}:", component.id)) {
                return Err(format!("occurrence '{path}' is not inside component {}", component.id));
            }
            let members = context.scene.occurrence_solids(path);
            if members.is_empty() {
                return Err(format!("occurrence '{path}' no longer exists"));
            }
            if !target.starts_with(&format!("{path}:")) && target != path {
                return Err(format!("the target is not part of occurrence '{path}'"));
            }
            (path.to_string(), members)
        }
        None => (component.id.clone(), context.scene.component_solids(&component.id)),
    };
    if members.is_empty() {
        return Err("component has no geometry".into());
    }
    let view = owning_view(annotation, context);
    let names: Vec<String> = members.iter().map(|(name, _)| name.clone()).collect();
    let stages = explode_stages(view, context, &names);
    let section = section_of(view);
    let exempt: Vec<String> = match view.and_then(|view| view.display.section.as_ref()) {
        Some(cut) => names.iter().filter(|name| cut.excludes(name)).cloned().collect(),
        None => Vec::new(),
    };
    let hidden: &[String] = view.map_or(&[], |view| view.display.hidden.as_slice());
    let visible: Vec<(String, u32)> = members
        .iter()
        .filter(|(name, _)| !hidden.contains(name))
        .cloned()
        .collect();
    // The DISPLAYED box of the occurrence: each member's box corners posed
    // through its own stages (so a half-exploded occurrence's bubble still
    // sits beside what is shown).
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for (name, handle) in if visible.is_empty() { &members } else { &visible } {
        let (lo, hi) = solid_bbox(*handle)?;
        let stages = stages.get(name).map_or(&[][..], Vec::as_slice);
        for corner in 0..8 {
            let point = [
                if corner & 1 == 0 { lo.x } else { hi.x },
                if corner & 2 == 0 { lo.y } else { hi.y },
                if corner & 4 == 0 { lo.z } else { hi.z },
            ];
            let posed = pose_apply(stages, point);
            low = Vec3::new(low.x.min(posed[0]), low.y.min(posed[1]), low.z.min(posed[2]));
            high = Vec3::new(high.x.max(posed[0]), high.y.max(posed[1]), high.z.max(posed[2]));
        }
    }
    let extent = high.sub(low);
    let reach = extent.x.max(extent.y).max(extent.z).max(1.0) * 0.35;
    let default_label = [high.x + reach, high.y + reach, (low.z + high.z) * 0.5];
    Ok(Occurrence {
        id,
        members,
        visible,
        stages,
        section,
        exempt,
        default_label,
    })
}

/// The nested occurrence path a balloon names, when it names one.
fn nested_path(annotation: &PmiAnnotation) -> Option<&str> {
    annotation
        .params
        .get("occurrence")
        .and_then(serde_json::Value::as_str)
        .map(str::trim)
        .filter(|path| !path.is_empty())
}

/// The `bom` record of the placement `path` names: the top-level component's
/// own for one segment, else the placing component's inside the owning
/// sub-assembly's definition, walked down the parts libraries.
fn placement_bom(context: &PmiContext<'_>, path: &str) -> Result<serde_json::Value, String> {
    let mut segments = path.split(':');
    let first = segments.next().unwrap_or_default();
    let mut params = context
        .request
        .features
        .iter()
        .find(|f| f.input_params["id"].as_str() == Some(first))
        .map(|f| f.input_params.clone())
        .ok_or("component no longer exists")?;
    // `None` is the open document's own library.
    let mut library: Option<serde_json::Value> = None;
    for segment in segments {
        let part = params["partName"].as_str().unwrap_or_default().to_string();
        let document = match &library {
            None => crate::feature_pipeline::parts_library::active_entry(&part)
                .map(|entry| entry.document)
                .or_else(|| context.request.parts_library.get(&part).map(|entry| entry.document.clone())),
            Some(library) => library.get(&part).map(|entry| entry["document"].clone()),
        }
        .ok_or_else(|| format!("occurrence '{path}': part '{part}' is not in the parts library"))?;
        params = document["features"]
            .as_array()
            .and_then(|features| features.iter().find(|f| f["inputParams"]["id"].as_str() == Some(segment)))
            .map(|f| f["inputParams"].clone())
            .ok_or_else(|| format!("occurrence '{path}' no longer exists"))?;
        library = Some(document["partsLibrary"].clone());
    }
    Ok(params["bom"].clone())
}

/// Where an error row's chip sits for a balloon without a stored bubble:
/// the default bubble beside the occurrence, when the occurrence resolves
/// at all — so a refused balloon is red NEXT TO its part, not at the origin.
pub fn error_bubble(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Option<[f64; 3]> {
    occurrence(annotation, context).ok().map(|occurrence| occurrence.default_label)
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("target");
    let occurrence = occurrence(annotation, context)?;
    let bom = placement_bom(context, &occurrence.id)?;
    let find = &bom["Find_Number"];
    let text = match find {
        serde_json::Value::String(s) => s.trim().to_owned(),
        serde_json::Value::Number(n) => n.to_string(),
        _ => String::new(),
    };
    if text.is_empty() {
        return Err("set the component's Find Number in the BOM before adding a balloon".into());
    }
    let component_id = occurrence.id.clone();
    if occurrence.visible.is_empty() {
        return Err(format!("component {component_id} is hidden in this view: show it or move the balloon to another view"));
    }
    let default_label = occurrence.default_label;
    // The bubble the head is derived from, in the displayed frame.
    let bubble = annotation.label_world.unwrap_or(default_label);
    let camera = owning_view(annotation, context).and_then(|view| view.camera.as_ref());
    let stages_of = |name: &str| occurrence.stages.get(name).map_or(&[][..], Vec::as_slice);
    let scope_of = |name: &str| HeadScope {
        stages: stages_of(name),
        section: occurrence.section.as_ref().filter(|_| !occurrence.exempt.iter().any(|exempt| exempt == name)),
    };
    // The anchor: nothing (the surface), a vertex ref, or an edge name — on
    // this occurrence, and on a member the view shows.
    let anchors = annotation.references("anchor");
    if anchors.len() > 1 {
        return Err("pick at most one anchor".into());
    }
    let owner_of = |name: &str| -> Result<(), String> {
        match context.scene.owning_component(name) {
            Some(owner) if owner.id == component_id => Ok(()),
            _ if component_id.contains(':') && name.starts_with(&format!("{component_id}:")) => Ok(()),
            _ => Err(format!("anchor '{name}' is not on component {component_id}")),
        }
    };
    let shown = |solid: &str, name: &str| -> Result<(), String> {
        if occurrence.visible.iter().any(|(member, _)| member == solid) {
            Ok(())
        } else {
            Err(format!("anchor '{name}' is on '{solid}', which is hidden in this view"))
        }
    };
    let (head, mode) = match anchors.first().map(String::as_str) {
        None => {
            let mut best: Option<(f64, [f64; 3])> = None;
            for (name, handle) in &occurrence.visible {
                let scope = scope_of(name);
                let candidate = crate::with_registered_solid_str(*handle, |solid| nearest_screen_point(solid, scope, bubble, camera))?;
                if let Some(display) = candidate {
                    let d2 = head_distance(camera, display, bubble);
                    if best.is_none_or(|(b, _)| d2 < b) {
                        best = Some((d2, display));
                    }
                }
            }
            let head = best.map(|(_, p)| p).ok_or_else(|| {
                format!("component {component_id} is sectioned away in this view: move the section or the balloon")
            })?;
            (head, BalloonAnchorMode::Surface)
        }
        Some(name) if name.contains('@') => {
            let (solid_name, position) = parse_vertex_ref(name)
                .ok_or_else(|| format!("anchor '{name}': a vertex ref is '{{solid}}@x,y,z'"))?;
            owner_of(solid_name)?;
            shown(solid_name, name)?;
            let handle = context
                .scene
                .resolve_solid(solid_name)
                .ok_or_else(|| format!("anchor '{name}': unknown solid '{solid_name}'"))?;
            // A vertex is picked on the DISPLAYED (posed) geometry; snap in
            // the modeling frame.
            let stages = stages_of(solid_name);
            let picked = v3(pose_unapply(stages, a3(position)));
            let geometry = crate::with_registered_solid_str(handle, |solid| Ok(resolve_vertex_selection(solid, picked)))?
                .map_err(|error| format!("anchor '{name}': {error}"))?;
            let SelectionGeometry::Point { position } = geometry else {
                return Err(format!("anchor '{name}' is not a vertex"));
            };
            let head = pose_apply(stages, a3(position));
            let exempt = occurrence.exempt.iter().any(|exempt| exempt == solid_name);
            if !exempt && occurrence.section.as_ref().is_some_and(|section| !section.keeps(head, 1e-7)) {
                return Err(format!("anchor '{name}' is sectioned away in this view"));
            }
            (head, BalloonAnchorMode::Point)
        }
        Some(name) => {
            let edge = context
                .scene
                .resolve_edge(name)
                .ok_or_else(|| format!("anchor '{name}' must be a vertex or an edge of component {component_id}"))?;
            owner_of(name)?;
            let solid_name = occurrence
                .members
                .iter()
                .find(|(_, handle)| *handle == edge.handle)
                .map(|(name, _)| name.clone())
                .ok_or_else(|| format!("anchor '{name}' is not on component {component_id}"))?;
            shown(&solid_name, name)?;
            let scope = scope_of(&solid_name);
            let head = crate::with_registered_solid_str(edge.handle, |solid| {
                let record = solid
                    .edges
                    .iter()
                    .find(|record| record.id == edge.edge_id)
                    .ok_or_else(|| format!("anchor edge '{name}' has no record"))?;
                nearest_displayed_point_on_edge(record, scope, bubble)
            })?
            .ok_or_else(|| format!("anchor '{name}' is sectioned away in this view"))?;
            (
                head,
                BalloonAnchorMode::Edge {
                    solid: solid_name,
                    edge_id: edge.edge_id,
                },
            )
        }
    };
    let solids: Vec<String> = occurrence.visible.iter().map(|(name, _)| name.clone()).collect();
    let poses: Vec<SolidPoses> = occurrence
        .stages
        .iter()
        .filter(|(_, stages)| !stages.is_empty())
        .map(|(solid, stages)| SolidPoses {
            solid: solid.clone(),
            stages: stages.clone(),
        })
        .collect();
    Ok(Resolved {
        text,
        value: None,
        unit: "",
        references: targets,
        geometry: PmiGeometry::Leader {
            targets: vec![head],
            dot: false,
            balloon: true,
            anchor: Some(BalloonAnchor {
                camera: camera.cloned(),
                solids,
                mode,
                poses,
                // A component the cut leaves whole re-projects uncut.
                section: occurrence.section.clone().filter(|_| occurrence.exempt.len() < occurrence.members.len()),
            }),
        },
        default_label,
    })
}
