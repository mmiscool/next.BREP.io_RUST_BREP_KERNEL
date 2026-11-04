//! SM Mirror: reflect a whole sheet-metal part about a plane or planar face.
//!
//! When the plane holds one of the base plate's side walls, the reflection
//! joins the part across that edge: one body, one flat, one flat pattern, and
//! walls that meet their own reflection in line become one wall
//! ([`SheetTree::mirror_joined`]). A plane clear of the part makes a separate
//! sheet-metal body; a plane through the part is refused.
use super::common;
use crate::feature_pipeline::sheet_metal::{self, SheetTree};
use crate::feature_pipeline::{FeatureContext, FeatureResult, SelectionProbe};

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.all_sheet_metal && probe.faces == 1 && probe.edges == 0 && probe.solids == 0
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(format!("sheet-metal mirror: {error}")),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    let Some(plane_name) = common::single_reference_name(ctx.param("mirrorPlane")) else {
        return Ok(result);
    };
    let Some(plane) = common::resolve_plane_reference(ctx, &plane_name) else {
        result.unresolved.push(plane_name);
        return Ok(result);
    };
    let (point, normal) = plane.point_normal()?;
    let normal = normal.normalized()?;
    // The part: the `sheet` pick, else the body the mirror face lies on, else
    // the only sheet-metal body in the scene.
    let face_body = ctx.scene.resolve_face(&plane_name).and_then(|face| {
        ctx.scene
            .solids
            .iter()
            .find(|(_, handle)| **handle == face.handle && sheet_metal::get_tree(face.handle).is_some())
            .map(|(name, _)| name.clone())
    });
    let Some(body) = common::first_reference_name(ctx.param("sheet"))
        .or(face_body)
        .or_else(|| common::sheet_body_name(ctx))
    else {
        return Err("select the sheet-metal part to mirror".into());
    };
    let Some(handle) = ctx.scene.resolve_solid(&body) else {
        result.unresolved.push(body);
        return Ok(result);
    };
    let tree: SheetTree = sheet_metal::get_tree(handle).ok_or("the selected body is not sheet metal")?;
    let tag = if ctx.id.is_empty() { "SM.MIRROR" } else { ctx.id.as_str() };
    if let Some(joined) = tree.mirror_joined(point, normal, tag)? {
        result.added.push(sheet_metal::register_folded(joined, &body)?);
        result.removed.push(body);
        return Ok(result);
    }
    // Not joinable: a separate copy, unless the plane cuts through the part.
    let (mut below, mut above) = (false, false);
    crate::with_registered_solid_str(handle, |solid| {
        for vertex in &solid.vertices {
            let side = vertex.point.sub(point).dot(normal);
            below |= side < -1e-7;
            above |= side > 1e-7;
        }
        Ok(())
    })?;
    if below && above {
        return Err("the mirror plane passes through the part; pick a plate side wall or a plane clear of the part".into());
    }
    let copy = tree
        .mirrored(point, normal, tag)?
        .ok_or("a part with flanges on hole rims cannot be mirrored yet")?;
    result.added.push(sheet_metal::register_folded(copy, &format!("{tag}:{body}:M"))?);
    Ok(result)
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type":"SM.MIRROR", "shortName":"SM.MI", "longName":"SM Mirror",
        "ribbonPath":"Home/Sheet Metal/Mirror", "commandSize":"Compact", "displayBuilder":false,
        "inputParamsSchema": {
            "id":{"type":"string","default_value":null},
            "mirrorPlane":{"label":"Mirror plane","type":"reference_selection","selectionFilter":["FACE","PLANE"],"multiple":false,"default_value":null,
                "hint":"Pick a side wall of the base plate to join the reflection into one part, or a plane clear of the part for a separate copy."},
            "sheet":{"label":"Part","type":"reference_selection","selectionFilter":["SOLID"],"multiple":false,"default_value":null,
                "hint":"The sheet-metal part. Optional when the mirror plane is one of its faces or it is the only sheet-metal part."}
        }
    })
}

