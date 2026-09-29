//! RFS — Refit Faces.
//!
//! Human-guided cleanup after a mesh import: the user picks faces the
//! automatic recognition left as separate facets and says what one surface
//! they are ("these strips are a cylinder"). The faces are replaced by ONE
//! face on that surface, fitted by the import's own RANSAC + least-squares
//! pass, and trimmed by the region's boundary rebuilt as exact curves.
//! Underlying kernel fn: [`crate::refit_faces_as_one`], which refuses by name
//! — faces not edge-connected, a fit residual over the tolerance, a region
//! that is not a disk or an annulus, a boundary with no exact curve — and
//! changes nothing when it does.
//!
//! Reference resolution (contract rule 1): `faces` is a `reference_selection`
//! of FACE names; a miss is an `unresolved` entry, never a panic. An empty
//! selection or faces from several solids is a soft no-op, as for Delete Face.
//!
//! Name fidelity (contract rule 2): the result reuses the TARGET solid's name.
//! The new face takes the name of the FIRST selected face; every neighbour
//! keeps its name; the other selected faces' names disappear.
//!
//! The fit is reported on success as a feature note ("fitted cylinder radius
//! …, residual max … rms …"), which the history panel shows under the feature.

use std::collections::HashSet;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureRefusal, FeatureResult};
use crate::{refit_faces_as_one, RefitSurfaceKind};

/// The tolerance when the field is empty: 0.1% of nothing in particular is
/// no default, so this is an absolute model-unit bar a scanned or exported
/// mesh in millimetres comfortably meets.
pub const DEFAULT_TOLERANCE: f64 = 0.01;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    let names = common::reference_name_array(ctx.param("faces"));
    if names.is_empty() {
        return Ok(result); // Soft no-op — nothing selected.
    }
    let mut resolved: Vec<(u32, u64)> = Vec::new();
    for name in &names {
        match ctx.scene.resolve_face(name) {
            Some(face_ref) => resolved.push((face_ref.handle, face_ref.face_id)),
            None => result.unresolved.push(name.clone()),
        }
    }
    if resolved.is_empty() {
        return Ok(result);
    }
    let handles: HashSet<u32> = resolved.iter().map(|(handle, _)| *handle).collect();
    if handles.len() > 1 {
        return Err("Refit Faces: the selected faces belong to more than one solid; pick faces of one body".into());
    }
    let target_handle = resolved[0].0;
    result.note_partial_resolution(&names);

    let kind_text = ctx.string("surfaceType").unwrap_or_default();
    let kind = RefitSurfaceKind::parse(&kind_text).ok_or_else(|| {
        format!(
            "Refit Faces: unknown surface type '{kind_text}' (auto, plane, cylinder, cone, sphere, torus)"
        )
    })?;
    let tolerance = match ctx.param("tolerance") {
        None | Some(serde_json::Value::Null) => DEFAULT_TOLERANCE,
        Some(_) => ctx.number("tolerance")?,
    };

    let mut seen: HashSet<u64> = HashSet::new();
    let face_ids: Vec<u64> = resolved
        .iter()
        .map(|(_, face_id)| *face_id)
        .filter(|face_id| seen.insert(*face_id))
        .collect();

    let target_name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, handle)| **handle == target_handle)
        .map(|(name, _)| name.clone())
        .ok_or("refit_faces: target solid has no scene-map name")?;

    let solid = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;
    let (solid, report) = refit_faces_as_one(&solid, &face_ids, kind, tolerance)?;
    result.notes.push(format!(
        "fitted {} to {} faces: residual max {:.3e}, rms {:.3e} (tolerance {})",
        report.carrier.describe(),
        report.replaced_faces,
        report.max_residual,
        report.rms_residual,
        report.tolerance
    ));

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
    result.removed.push(target_name);
    Ok(result)
}

/// Context-bar applicability: selected faces drive `faces` — the user's own
/// flow, "select these faces, then say what they are".
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "RFS",
    "shortName": "RFS",
    "longName": "Refit Faces",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the refit faces feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "Select the edge-connected faces that together are really ONE surface (typically the facets or strips a mesh import left behind). They are replaced by a single face on the fitted surface"
        },
        "surfaceType": {
            "type": "options",
            "options": [
                "AUTO",
                "PLANE",
                "CYLINDER",
                "CONE",
                "SPHERE",
                "TORUS"
            ],
            "default_value": "AUTO",
            "hint": "The kind of surface the selected faces form. AUTO takes the best fit among the five"
        },
        "tolerance": {
            "type": "number",
            "default_value": DEFAULT_TOLERANCE,
            "hint": "Largest distance any vertex of the selected faces may lie from the fitted surface. A fit that misses by more is refused, and the refusal says by how much"
        }
    }
})
}

