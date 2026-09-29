//! Split solids with a base plane, planar face, or supported analytic face carrier.
//!
//! Pieces replace the source and retain surviving face names. Names use
//! `${featureID}:${solidName}:A` for above/outside and `:B` for below/inside;
//! keeping both emits A before B. Missing body or face references are reported
//! as unresolved. Unsupported carriers and nonintersecting cuts return errors.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::{
    split_solid_by_face_surface, split_solid_by_plane, AnalyticSurface, BrepSolid, NurbsSurface,
    Vec3,
};

/// Base-plane normals through the world origin: the cut
/// plane is the named base plane, its normal the axis PERPENDICULAR to it.
fn plane_normal(key: &str) -> Vec3 {
    match key {
        "YZ" => Vec3::new(1.0, 0.0, 0.0),
        "XZ" => Vec3::new(0.0, 1.0, 0.0),
        _ => Vec3::new(0.0, 0.0, 1.0), // XY (default)
    }
}

/// The resolved cut tool, shared across every solid being split.
enum CutTool {
    /// A plane through `point` with unit `normal` → `split_solid_by_plane`.
    Plane { point: Vec3, normal: Vec3 },
    /// A non-planar analytic carrier → `split_solid_by_face_surface`.
    Surface(NurbsSurface),
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // `solids` is a reference_selection of solid names (contract rule 1). No
    // selection → no-op.
    let names = common::reference_name_array(ctx.param("solids"));
    if names.is_empty() {
        return Ok(result);
    }

    let resolved = common::resolve_solid_names(ctx.scene, &names, &mut result.unresolved);

    // Resolve the cut tool (may add its own `unresolved` entry for a missing
    // planeRef face → no-op, per contract rule 1 / mirror's precedent).
    let tool = match resolve_cut_tool(ctx, &mut result)? {
        Some(tool) => tool,
        None => return Ok(result),
    };

    if resolved.is_empty() {
        return Ok(result);
    }

    let feature_id = if ctx.id.is_empty() { "SPLIT" } else { ctx.id.as_str() };
    let keep = ctx
        .string("keep")
        .unwrap_or_default()
        .to_uppercase();
    let keep = if keep.is_empty() { "BOTH".to_string() } else { keep };
    let keep_above = keep == "BOTH" || keep == "ABOVE";
    let keep_below = keep == "BOTH" || keep == "BELOW";

    for (solid_name, handle) in &resolved {
        // Short registry borrow: run the split and hand back the owned pieces.
        let pieces = crate::with_registered_solid_str(*handle, |solid| split(solid, &tool));
        let pieces = match pieces {
            Ok(pieces) => pieces,
            Err(error) => {
                // Handle hygiene: free any pieces already registered for earlier
                // solids before halting (a leaked handle is a wasm32 OOM).
                free_added(&result.added);
                return Err(error);
            }
        };
        let [below, above] = pieces;

        // A(above/+n/outside) = pieces[1]; B(below/−n/inside) = pieces[0]. Keep
        // selects; when BOTH, A is added before B.
        if keep_above {
            result.added.push(register_piece(
                above,
                &format!("{feature_id}:{solid_name}:A"),
                &format!("{feature_id}:A"),
            ));
        }
        if keep_below {
            result.added.push(register_piece(
                below,
                &format!("{feature_id}:{solid_name}:B"),
                &format!("{feature_id}:B"),
            ));
        }
        // The cut consumes the original body.
        result.removed.push(solid_name.clone());
    }

    if result.added.is_empty() {
        // Every kept piece was excluded by `keep` yet a solid resolved — the caller
        // throws "failed to process any selected solids".
        return Err("split: `keep` excluded every piece".into());
    }

    Ok(result)
}

/// Run the split, normalizing both kernel entries to `[below, above]`.
fn split(solid: &BrepSolid, tool: &CutTool) -> Result<[BrepSolid; 2], String> {
    match tool {
        CutTool::Plane { point, normal } => {
            let (below, above) = split_solid_by_plane(solid, *point, *normal)?;
            Ok([below, above])
        }
        CutTool::Surface(surface) => {
            let pieces = split_solid_by_face_surface(solid, surface)?;
            let mut pieces = pieces.into_iter();
            let below = pieces
                .next()
                .ok_or("split: surface cut produced no pieces")?;
            let above = pieces
                .next()
                .ok_or("split: surface cut produced only one piece")?;
            Ok([below, above])
        }
    }
}

/// Register a kept piece under `name`. A cut divides shared faces, so BOTH
/// halves would otherwise inherit the source's face/edge names verbatim (two
/// resident solids sharing a name — the exact collision the naming-contract
/// guard forbids). `namespace_copy_names` suffixes the piece's copy of those
/// names with the per-invocation `suffix` (`{feature_id}:A` / `:B`), keeping
/// them unique and mode-independent (the same piece is named the same whether
/// or not its sibling survives).
fn register_piece(mut piece: BrepSolid, name: &str, suffix: &str) -> AddedSolid {
    common::namespace_copy_names(&mut piece, suffix);
    let face_names = common::collect_face_names(&piece);
    let edge_names = common::collect_edge_names(&piece);
    let handle = crate::register_solid_value(piece);
    AddedSolid {
        handle,
        name: name.to_string(),
        face_names,
        edge_names,
        ..AddedSolid::default()
    }
}

/// Free already-registered pieces before a mid-loop error halt (no handle leak).
fn free_added(added: &[AddedSolid]) {
    for piece in added {
        crate::free_registered_solid(piece.handle);
    }
}

/// Resolve the cut tool:
/// 1. a `planeRef` face NAME resolved against the scene-map:
///    - a PLANAR face → cut by its plane, shifted `normal·offset` (the caller
///      `resolvePlaneReference(refObj, offset)`);
///    - a non-planar analytic face → cut by its carrier surface (the caller ignores
///      `offset` on this path — matched, with the note below);
///    - a name that does not resolve → an `unresolved` no-op (contract rule 1).
/// 2. else the named base plane (`plane` param + `offset·normal`).
fn resolve_cut_tool(
    ctx: &FeatureContext,
    result: &mut FeatureResult,
) -> Result<Option<CutTool>, String> {
    let offset = match ctx.param("offset") {
        None | Some(serde_json::Value::Null) => 0.0,
        Some(_) => ctx.number("offset")?,
    };

    if let Some(name) = plane_ref_name(ctx.param("planeRef")) {
        return match common::resolve_plane_reference(ctx, &name) {
            // A FACE keeps the curved-carrier path: a planar face → its offset
            // plane, a curved analytic face → its carrier surface.
            Some(common::PlaneLikeRef::Face(face_ref)) => {
                let tool = crate::with_registered_solid_str(face_ref.handle, |solid| {
                    cut_tool_from_face(solid, face_ref.face_id, offset)
                })?;
                Ok(Some(tool))
            }
            // A datum / construction PLANE → an offset plane cut (its frame's
            // origin + z-axis normal, shifted by `offset`).
            Some(common::PlaneLikeRef::Frame(frame)) => Ok(Some(CutTool::Plane {
                point: frame.origin.add(frame.z_axis.scale(offset)),
                normal: frame.z_axis,
            })),
            None => {
                result.unresolved.push(name);
                Ok(None)
            }
        };
    }

    // Base plane: plane_point = origin + normal·offset.
    let key = ctx
        .string("plane")
        .unwrap_or_default()
        .to_uppercase();
    let normal = plane_normal(&key);
    Ok(Some(CutTool::Plane {
        point: normal.scale(offset),
        normal,
    }))
}

/// Build a [`CutTool`] from a referenced face: a planar face → its offset plane;
/// any other analytic face → its carrier surface (offset ignored, as before).
fn cut_tool_from_face(
    solid: &BrepSolid,
    face_id: u64,
    offset: f64,
) -> Result<CutTool, String> {
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.id != face_id {
                continue;
            }
            if matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
                // Planar face → its plane, shifted along the face normal by
                // `offset` (`resolvePlaneReference`). Point/normal from the
                // surface midpoint, oriented by `same_sense` (mirror's pattern).
                let [u0, u1] = face.surface.domain_u()?;
                let [v0, v1] = face.surface.domain_v()?;
                let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
                let point = face.surface.evaluate(um, vm)?;
                let mut normal = face.surface.normal(um, vm)?;
                if !face.same_sense {
                    normal = normal.scale(-1.0);
                }
                return Ok(CutTool::Plane {
                    point: point.add(normal.scale(offset)),
                    normal,
                });
            }
            // Non-planar analytic carrier: cut by the surface itself. The kernel
            // recognizes cylinder/cone/sphere/torus and extends the carrier;
            // general revolutions error there (deferred). `offset` has no
            // meaning for a curved carrier — ignored here.
            return Ok(CutTool::Surface(face.surface.clone()));
        }
    }
    Err(format!("split: planeRef face id {face_id} not found"))
}

/// The `planeRef` reference name (`multiple: false`; a string, a `{ name }`
/// object, or the first entry of a saved singleton array). Empty → `None`.
fn plane_ref_name(value: Option<&serde_json::Value>) -> Option<String> {
    let name = match value {
        Some(serde_json::Value::String(text)) => Some(text.clone()),
        Some(serde_json::Value::Array(array)) => array.iter().find_map(|entry| {
            entry
                .as_str()
                .or_else(|| entry.get("name").and_then(|name| name.as_str()))
                .map(str::to_string)
        }),
        Some(object @ serde_json::Value::Object(_)) => object
            .get("name")
            .and_then(|name| name.as_str())
            .map(str::to_string),
        _ => None,
    }?;
    let trimmed = name.trim();
    (!trimmed.is_empty()).then(|| trimmed.to_string())
}



/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected solids drive `solids`, a face `planeRef`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0 || probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SPL",
    "shortName": "SPL",
    "longName": "Split",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the split feature"
        },
        "solids": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select one or more solids to cut by the plane"
        },
        "plane": {
            "type": "options",
            "options": [
                "XY",
                "YZ",
                "XZ"
            ],
            "default_value": "XY",
            "hint": "Base plane to cut with (through the world origin unless offset). Ignored when a plane reference is selected."
        },
        "offset": {
            "type": "number",
            "default_value": 0,
            "hint": "Shift the cut plane along its normal (cut at origin + normal·offset)"
        },
        "planeRef": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "PLANE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Optional face or datum to cut with (overrides the base plane). A planar face cuts by its plane; a cylindrical / conical / spherical face cuts by that analytic surface (Golovanov §6.4)."
        },
        "keep": {
            "type": "options",
            "options": [
                "BOTH",
                "ABOVE",
                "BELOW"
            ],
            "default_value": "BOTH",
            "hint": "Which piece(s) to keep: BOTH halves, only ABOVE (+normal side), or only BELOW (−normal side)"
        }
    }
})
}

