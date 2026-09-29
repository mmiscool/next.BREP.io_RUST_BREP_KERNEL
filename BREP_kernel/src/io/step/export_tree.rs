//! Build the STRUCTURED STEP export tree out of the document's parts library.
//!
//! [`assembly_export_tree`] is the consumer half of `assembly.rs`: it turns the
//! app's assembly model — a parts library of unique parts plus ACOMP instances
//! that place them — into the [`StepAssemblyExport`] the writer emits.
//!
//! # Where each half comes from
//!
//! * **Structure** is read from the DOCUMENTS. The root's occurrences are passed
//!   in (the app holds the live, post-solve component poses); every level below
//!   comes from that part's own embedded document, whose ACOMP features carry
//!   `partName` and the `{translate, rotateEulerDeg}` pose. Those values are
//!   expression-capable, so they are evaluated through the part document's OWN
//!   [`Env`] and composed by [`compose_trs_matrix`] — the same two calls the
//!   ACOMP feature makes, never a re-derived convention.
//! * **Geometry** is read from the SNAPSHOTS. A library entry's snapshot is the
//!   evaluated part in its OWN LOCAL FRAME, which is exactly what a STEP product
//!   must carry. Nothing is re-executed and nothing is un-posed: taking the
//!   scene's placed members and multiplying by the inverse pose would reproduce
//!   the snapshot only to rounding, and would not separate a sub-assembly's own
//!   bodies from its children's.
//!
//! # Splitting a sub-assembly's snapshot
//!
//! A sub-assembly's snapshot holds its children's bodies too, because the ACOMP
//! features inside it ran when the snapshot was built — and those arrive
//! NAMESPACED (`ACOMP1:Extrude1`, chaining further down). So a product's OWN
//! bodies are the snapshot solids whose name carries no `ACOMP<digits>:` prefix;
//! the prefixed ones are the child products' business and are dropped here, to
//! be written once, unposed, in the child's own product.
//!
//! # Dedup
//!
//! One product per library entry PER DOCUMENT LEVEL: two instances of `bolt` in
//! one assembly share a product and get two occurrences, which is the whole
//! point. A part reached from two different parents gets a product each — the
//! rigid-nesting model already stores it once per level, so there is no content
//! identity to dedup across levels.
//!
//! # Identity
//!
//! `PRODUCT.name` is the parts-library entry name. `PRODUCT.id` is the part's
//! BOM part number — the `Part_Number` field of the embedded document's
//! top-level [`PART_ATTRIBUTES`] record, the same record the BOM panel's
//! `part.` columns and the Part Properties dialog read and write — because that
//! is the identity a downstream PDM keys on. A part with no part number falls
//! back to the library entry's `sourceKey` (the file it was imported from), and
//! a part with neither writes an empty id, which the writer fills with the
//! name. The ROOT product's id is empty: [`assembly_export_tree`] never sees
//! the root document, only its name.

use std::collections::BTreeMap;

use super::assembly::{StepAssemblyExport, StepExportOccurrence, StepExportProduct};
use super::Mat4;
use crate::feature_pipeline::features::common::{compose_trs_matrix, vec3_from_value};
use crate::feature_pipeline::parts_library::{self, PartsLibraryEntry, PartsLibraryMap};
use crate::feature_pipeline::{Env, HistoryRequest};
use crate::{is_component_reference, restore_solids, BrepSolid};

/// How deep the recursion will go before it refuses. Matches the import side's
/// own cap (`model_io.rs::MAX_NESTED_DEPTH`): sixty-four levels of embedded
/// documents is far past anything a real assembly carries, and the guard is
/// what keeps a malformed (but acyclic) document off the native stack.
const MAX_DEPTH: usize = 64;

/// The top-level document key holding a part's own BOM attribute record
/// (`{ "Part_Number": "PN-7", "Material": "6061", ... }`). Owned by the app's
/// document model; the kernel reads it only to stamp `PRODUCT.id`.
pub const PART_ATTRIBUTES: &str = "partAttributes";
/// The BOM part-number field inside [`PART_ATTRIBUTES`].
pub const PART_NUMBER: &str = "Part_Number";

/// A part document's BOM part number: `partAttributes.Part_Number`, trimmed;
/// `None` when the record, the field, or its text is absent or blank.
fn part_number(document: &serde_json::Value) -> Option<String> {
    document
        .get(PART_ATTRIBUTES)?
        .get(PART_NUMBER)?
        .as_str()
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .map(str::to_string)
}

/// The component namespace prefix a scene name opens with, if any:
/// `ACOMP3:Extrude1` -> `Some("ACOMP3")`, `Extrude1` -> `None`.
fn component_prefix(name: &str) -> Option<&str> {
    let (head, _) = name.split_once(':')?;
    is_component_reference(head).then_some(head)
}

/// Build the export tree for a document with components.
///
/// * `document_name` names the ROOT product.
/// * `root_bodies` is every resident solid of the document with its scene name;
///   the component-owned ones are filtered out here, so the caller passes the
///   scene exactly as it has it.
/// * `root_components` is one entry per ACOMP instance of the ROOT document:
///   `(component id, parts-library entry name, instance pose)`. The pose is the
///   live one, so a solved or gizmo-moved instance exports where it sits.
pub fn assembly_export_tree(
    document_name: &str,
    root_bodies: Vec<(String, BrepSolid)>,
    root_components: &[(String, String, Mat4)],
) -> Result<StepAssemblyExport, String> {
    let own: Vec<(String, BrepSolid)> = root_bodies
        .into_iter()
        .filter(|(name, _)| component_prefix(name).is_none())
        .collect();
    let mut assembly = StepAssemblyExport {
        products: vec![StepExportProduct {
            name: document_name.to_string(),
            id: String::new(),
            bodies: own,
            // The ROOT document's own PMI is not a product's: it rides the
            // export's `pmi` argument, resolved against the live scene, and is
            // written through the occurrence paths (`assembly.rs` step 7b).
            pmi: None,
        }],
        occurrences: Vec::new(),
    };
    let library = parts_library::parts_library_map();
    place_children(&mut assembly, 0, &library, root_components, 0)?;
    Ok(assembly)
}

/// Add one occurrence per instance, building each distinct part ONCE at this
/// level (`seen`) so repeated placements share a product.
fn place_children(
    assembly: &mut StepAssemblyExport,
    parent: usize,
    library: &PartsLibraryMap,
    instances: &[(String, String, Mat4)],
    depth: usize,
) -> Result<(), String> {
    let mut seen: BTreeMap<&str, usize> = BTreeMap::new();
    for (designator, part_name, placement) in instances {
        let child = match seen.get(part_name.as_str()) {
            Some(index) => *index,
            None => {
                let entry = library.get(part_name).ok_or_else(|| {
                    format!(
                        "export_step: part '{part_name}' (placed by {designator}) is not in \
                         the parts library of '{}'",
                        assembly.products[parent].name
                    )
                })?;
                let index = add_part(assembly, part_name, entry, depth)?;
                seen.insert(part_name.as_str(), index);
                index
            }
        };
        assembly.occurrences.push(StepExportOccurrence {
            designator: designator.clone(),
            parent,
            child,
            placement: *placement,
        });
    }
    Ok(())
}

/// Add one parts-library entry as a product, then recurse into the components
/// ITS document places. Returns the new product's index.
fn add_part(
    assembly: &mut StepAssemblyExport,
    part_name: &str,
    entry: &PartsLibraryEntry,
    depth: usize,
) -> Result<usize, String> {
    if depth >= MAX_DEPTH {
        return Err(format!(
            "export_step: assembly nests deeper than {MAX_DEPTH} levels at part '{part_name}'"
        ));
    }
    // The snapshot is the fast lane the ACOMP feature itself takes; a stale or
    // unreadable one heals the same way, by re-executing the embedded document.
    let restored = match restore_solids(&entry.snapshot) {
        Ok(restored) if !entry.dirty => restored,
        _ => {
            let (snapshot, _) = parts_library::rebuild_snapshot(&entry.document)
                .map_err(|error| format!("export_step: part '{part_name}': {error}"))?;
            restore_solids(&snapshot)
                .map_err(|error| format!("export_step: part '{part_name}': {error}"))?
        }
    };
    // This product's OWN bodies: the snapshot solids that no nested component
    // contributed. The namespaced ones are written in the child's product.
    let bodies: Vec<(String, BrepSolid)> = restored
        .solids
        .into_iter()
        .filter(|solid| component_prefix(&solid.name).is_none())
        .map(|solid| (solid.name, solid.solid))
        .collect();
    let index = assembly.products.len();
    // The BOM part number is the identity a PDM reads; a part that was never
    // given one is still identified by the file it came from.
    let id = part_number(&entry.document).unwrap_or_else(|| entry.source_key.clone());
    // The part's OWN PMI, resolved against the part's own geometry. Read from
    // the embedded document rather than the snapshot, because an annotation is
    // a reference plus a value, not geometry — and resolved here, where the
    // part document is in hand, so the writer can put it in the part's product
    // instead of the assembly's (`assembly.rs` step 7a).
    let pmi = parts_library::document_pmi(&entry.document);
    assembly.products.push(StepExportProduct {
        name: part_name.to_string(),
        id,
        bodies,
        pmi,
    });
    let (library, instances) = document_components(&entry.document, part_name)?;
    place_children(assembly, index, &library, &instances, depth + 1)?;
    Ok(index)
}

/// A part document's own parts library and its ACOMP instances, with each pose
/// evaluated in THAT document's expression environment and composed through the
/// one feature-transform convention.
fn document_components(
    document: &serde_json::Value,
    part_name: &str,
) -> Result<(PartsLibraryMap, Vec<(String, String, Mat4)>), String> {
    let request: HistoryRequest = serde_json::from_value(document.clone()).map_err(|error| {
        format!("export_step: part '{part_name}' document does not parse: {error}")
    })?;
    let env = Env::build(&request.expressions, &request.configurator).unwrap_or_else(Env::poisoned);
    let mut instances = Vec::new();
    for feature in &request.features {
        if !parts_library::is_acomp_type(&feature.feature_type) {
            continue;
        }
        let params = &feature.input_params;
        let string = |key: &str| {
            params
                .get(key)
                .and_then(serde_json::Value::as_str)
                .map(str::trim)
                .filter(|text| !text.is_empty())
                .map(str::to_string)
        };
        let Some(id) = string("id") else {
            return Err(format!(
                "export_step: part '{part_name}' has a component with no id"
            ));
        };
        let Some(child) = string("partName") else {
            return Err(format!(
                "export_step: part '{part_name}' component {id} names no part"
            ));
        };
        let transform = params.get("transform");
        let translate = vec3_from_value(
            &env,
            transform.and_then(|value| value.get("translate")),
            "transform.translate",
            [0.0, 0.0, 0.0],
        )?;
        let rotate_deg = vec3_from_value(
            &env,
            transform.and_then(|value| value.get("rotateEulerDeg")),
            "transform.rotateEulerDeg",
            [0.0, 0.0, 0.0],
        )?;
        let placement = compose_trs_matrix(
            translate,
            [
                rotate_deg[0].to_radians(),
                rotate_deg[1].to_radians(),
                rotate_deg[2].to_radians(),
            ],
            [1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0],
        );
        instances.push((id, child, placement));
    }
    Ok((request.parts_library, instances))
}

