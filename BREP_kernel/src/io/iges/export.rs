//! Kernel `BrepSolid` → IGES trimmed-NURBS-surface export.
//!
//! Each B-rep face becomes one IGES **144** Trimmed Surface: a **128** rational
//! B-spline surface, and one **142** Curve-on-Surface per loop (outer first).
//! Each 142 references two **102** composite curves — the loop's coedge pcurves
//! in parameter space (`BPTR`) and the same boundary as 3D edge curves
//! (`CPTR`) — assembled from per-coedge **126** rational B-spline curves.
//!
//! Face orientation (`same_sense`) is intentionally NOT encoded: the importer
//! recovers a coherent outward orientation with [`crate::sew_solid`], so the
//! wire format need only carry surfaces and boundary geometry.

use crate::step::edge_subcurve;
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord, LoopRecord};

use super::entities::{
    composite_102, curve_on_surface_142, curve_to_126, surface_to_128, trimmed_surface_144,
};
use super::writer::{GlobalParams, IgesWriter};

/// Serialize solids as an IGES 5.3 document of trimmed NURBS surfaces.
///
/// `unit` is one of `millimeter|meter|centimeter|inch|foot` (default
/// millimetre); coordinates are written verbatim, so callers should pass the
/// unit the kernel geometry is expressed in (millimetres in this kernel).
pub fn export_iges(
    solids: &[BrepSolid],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    if solids.is_empty() {
        return Err("iges_export: no solids to export".into());
    }
    let (units_flag, units_name) = units_for(unit);

    let mut writer = IgesWriter::new();
    writer.add_start_line(&format!("IGES export of '{name}' from the BREP kernel."));

    let mut face_count = 0usize;
    for solid in solids {
        for shell in &solid.shells {
            for face in &shell.faces {
                export_face(&mut writer, solid, face)?;
                face_count += 1;
            }
        }
    }
    if face_count == 0 {
        return Err("iges_export: solids contain no faces".into());
    }

    let global = GlobalParams {
        product_id: if name.is_empty() { "PART".into() } else { name.to_string() },
        file_name: format!("{}.igs", if name.is_empty() { "part" } else { name }),
        units_flag,
        units_name,
        timestamp: normalize_timestamp(timestamp),
        min_resolution: 1e-7,
    };
    Ok(writer.finish(global))
}

fn export_face(writer: &mut IgesWriter, solid: &BrepSolid, face: &FaceRecord) -> Result<(), String> {
    if face.loops.is_empty() {
        return Err(format!("iges_export: face {} has no loops", face.id));
    }
    let sptr = writer.add_entity(surface_to_128(&face.surface)?);

    let mut loop_ptrs: Vec<i64> = Vec::with_capacity(face.loops.len());
    for loop_record in &face.loops {
        loop_ptrs.push(export_loop(writer, solid, sptr, loop_record)?);
    }
    let (outer, holes) = loop_ptrs
        .split_first()
        .ok_or_else(|| format!("iges_export: face {} produced no boundaries", face.id))?;
    writer.add_entity(trimmed_surface_144(sptr, *outer, holes));
    Ok(())
}

/// Export one loop as a 142 (returns its DE pointer). Emits per-coedge 126
/// curves in both parameter and model space, grouped by two 102 composites.
fn export_loop(
    writer: &mut IgesWriter,
    solid: &BrepSolid,
    sptr: i64,
    loop_record: &LoopRecord,
) -> Result<i64, String> {
    if loop_record.coedges.is_empty() {
        return Err(format!("iges_export: loop {} has no coedges", loop_record.id));
    }
    let mut model_ptrs: Vec<i64> = Vec::with_capacity(loop_record.coedges.len());
    let mut param_ptrs: Vec<i64> = Vec::with_capacity(loop_record.coedges.len());
    for coedge in &loop_record.coedges {
        let edge = edge_for(solid, coedge.edge_id)?;
        // 3D edge curve, trimmed to the represented interval and oriented in
        // loop direction.
        let mut model_curve = edge_subcurve(edge)?;
        if !coedge.forward {
            model_curve = model_curve.reversed()?;
        }
        model_ptrs.push(writer.add_entity(curve_to_126(&model_curve)?));
        // Parameter-space pcurve is already directional.
        param_ptrs.push(writer.add_entity(curve_to_126(&coedge.pcurve)?));
    }
    let model_composite = writer.add_entity(composite_102(&model_ptrs));
    let param_composite = writer.add_entity(composite_102(&param_ptrs));
    Ok(writer.add_entity(curve_on_surface_142(sptr, param_composite, model_composite)))
}

fn edge_for<'a>(solid: &'a BrepSolid, id: u64) -> Result<&'a EdgeRecord, String> {
    solid
        .edges
        .iter()
        .find(|e| e.id == id)
        .ok_or_else(|| format!("iges_export: dangling edge id {id}"))
}

fn units_for(unit: &str) -> (i64, String) {
    match unit.trim().to_ascii_lowercase().as_str() {
        "inch" | "inches" | "in" => (1, "IN".into()),
        "foot" | "feet" | "ft" => (4, "FT".into()),
        "meter" | "metre" | "m" => (6, "M".into()),
        "centimeter" | "centimetre" | "cm" => (10, "CM".into()),
        _ => (2, "MM".into()),
    }
}

fn normalize_timestamp(timestamp: &str) -> String {
    if timestamp.trim().is_empty() {
        "00000000.000000".into()
    } else {
        timestamp.to_string()
    }
}
