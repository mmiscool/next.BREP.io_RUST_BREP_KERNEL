//! Edge and face names derived from the faces that meet them.

use crate::topology::*;
use crate::*;


/// Collect up to two distinct face names per edge, preserving first encounter order.
pub fn collect_edge_face_names(
    solid: &BrepSolid,
) -> (std::collections::HashMap<u64, Vec<String>>, Vec<u64>) {
    use std::collections::HashMap;
    let mut edge_faces: HashMap<u64, Vec<String>> = HashMap::new();
    let mut encounter_order: Vec<u64> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            let face_name = face.name.clone().unwrap_or_default();
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    let entry = edge_faces.entry(coedge.edge_id).or_insert_with(|| {
                        encounter_order.push(coedge.edge_id);
                        Vec::new()
                    });
                    if entry.len() < 2 && !entry.contains(&face_name) {
                        entry.push(face_name.clone());
                    }
                }
            }
        }
    }
    (edge_faces, encounter_order)
}

/// Name topology edges `{faceA}|{faceB}[n]`, sorting adjacent face names and
/// numbering each pair in shell/face/loop encounter order. Existing names that
/// contain `|` are regenerated; authored names are preserved.
pub fn stamp_derived_edge_names(solid: &mut BrepSolid) {
    use std::collections::HashMap;
    let (edge_faces, encounter_order) = collect_edge_face_names(solid);
    let solid_name_fallback = "Solid".to_string();
    let mut base_counts: HashMap<String, usize> = HashMap::new();
    for edge_id in encounter_order {
        let Some(edge) = solid.edges.iter_mut().find(|edge| edge.id == edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let mut faces: Vec<String> = edge_faces
            .get(&edge_id)
            .map(|list| list.iter().filter(|n| !n.is_empty()).cloned().collect())
            .unwrap_or_default();
        faces.sort();
        let base = if faces.len() >= 2 {
            format!("{}|{}", faces[0], faces[1])
        } else {
            format!(
                "{}|BOUNDARY",
                faces.first().unwrap_or(&solid_name_fallback)
            )
        };
        // Only topology edges advance the counter, keeping each pair's indices dense.
        let is_topology_name = edge.name.as_deref().is_none_or(|n| n.contains('|'));
        if is_topology_name {
            let index = base_counts.entry(base.clone()).or_insert(0);
            edge.name = Some(format!("{base}[{index}]"));
            *index += 1;
        }
    }
}

/// Disambiguate duplicate face names with `[n]` in shell/face encounter order.
/// Unique names stay unchanged. Run before [`stamp_derived_edge_names`] so edge
/// names embed the final face names and scene lookups remain unambiguous.
pub fn ensure_unique_face_names(solid: &mut BrepSolid) {
    use std::collections::HashMap;
    let mut counts: HashMap<String, usize> = HashMap::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if let Some(name) = &face.name {
                *counts.entry(name.clone()).or_insert(0) += 1;
            }
        }
    }
    let mut running: HashMap<String, usize> = HashMap::new();
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            let Some(name) = face.name.clone() else { continue };
            if counts.get(&name).copied().unwrap_or(0) > 1 {
                let index = running.entry(name.clone()).or_insert(0);
                face.name = Some(format!("{name}[{index}]"));
                *index += 1;
            }
        }
    }
}
