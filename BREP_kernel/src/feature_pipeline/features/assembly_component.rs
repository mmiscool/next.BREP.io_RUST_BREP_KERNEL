//! ACOMP — Assembly Component: one placed INSTANCE of a parts-library
//! entry. FULLY IMPLEMENTED.
//!
//! `inputParams = { partName, transform, isFixed }` — a reference into the
//! parts library (`parts_library.rs`) with NO embedded payload. `transform` is
//! the rigid instance pose `{ translate: [x,y,z], rotateEulerDeg: [rx,ry,rz] }`,
//! expression-capable per component, composed through the ONE feature-transform
//! convention ([`common::compose_trs_matrix`], intrinsic-XYZ Euler — the same
//! compose XFORM uses and the same one any solver/gizmo pose write-back must
//! use). Namespacing and registration are `component::create_component`'s job;
//! this feature only chooses WHERE the part-local solids come from:
//!
//! - **FAST (default):** decode the library entry's `snapshot` payload straight
//!   into kernel structs — no sub-part history execution.
//! - **SELF-HEAL:** an unreadable/corrupt snapshot (or a DIRTY entry — the
//!   `refresh_library_entry` / edit-in-context signal) re-executes the entry's
//!   embedded `document` in isolation, REWRITES the snapshot, then proceeds as
//!   FAST.
//! - **STALE:** a snapshot made by a different kernel build (its
//!   `snapshotProducer` stamp differs from this build's, or it has none) is
//!   rebuilt the same way. The rebuild is never silent and never destructive:
//!   when it changes the part, the feature's `notes` say by how much, and when
//!   it FAILS the stored snapshot is used with a note saying so, because a
//!   document that loads today must not stop loading because the kernel moved. The history cache re-runs this feature when the entry's content
//!   changes (see `parts_library::mix_descriptor_fingerprint`).
//!
//! **Auto-ground:** when `isFixed` is ABSENT and no component exists earlier in
//! the history, the instance is grounded (the first component of an empty
//! assembly is auto-fixed). An EXPLICIT `false` is always respected — the rule
//! keys on absence, so un-fixing the sole component stays possible. Wave-3
//! seam: the insert flow should write the resolved value into `inputParams` so
//! it is visible/editable in the dialog.
//!
//! Metadata: the snapshot's captured records are stamped under the NAMESPACED
//! names (`{id}:{name}`), replacing wholesale each run (a refreshed part's
//! dropped keys must not linger); an `inheritsFrom` value is namespaced too so
//! a sub-part's inheritance chain stays inside the sub-part.

use std::collections::BTreeMap;

use crate::feature_pipeline::component::{self, create_component, NestedConstraint};
use crate::feature_pipeline::features::common;
use crate::feature_pipeline::ports;
use crate::feature_pipeline::{
    parts_library, scene_metadata, FeatureContext, FeatureResult, PortRecord,
};
use crate::{restore_solids, AffineTransform, RestoredSnapshot};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // The feature id is the namespace prefix AND the syntactic component
    // reference the resolver/constraint lanes parse (`ACOMP<digits>`), so an
    // off-pattern id must fail loudly here — `split_component_namespace` would
    // silently fail to strip it later.
    if !crate::is_component_reference(&ctx.id) {
        return Err(format!(
            "component feature id '{}' must have the form ACOMP<digits>",
            ctx.id
        ));
    }
    let part_name = ctx
        .string("partName")
        .map(|name| name.trim().to_string())
        .filter(|name| !name.is_empty())
        .ok_or("missing required param `partName` (a parts-library entry name)")?;
    let entry = parts_library::active_entry(&part_name).ok_or_else(|| {
        format!("part '{part_name}' is not in the parts library")
    })?;

    // The rigid instance pose (expression-capable vec3s, ONE shared compose).
    let transform_value = ctx.param("transform");
    let translate = common::vec3_from_value(
        ctx.env,
        transform_value.and_then(|value| value.get("translate")),
        "transform.translate",
        [0.0, 0.0, 0.0],
    )?;
    let rotate_deg = common::vec3_from_value(
        ctx.env,
        transform_value.and_then(|value| value.get("rotateEulerDeg")),
        "transform.rotateEulerDeg",
        [0.0, 0.0, 0.0],
    )?;
    let matrix = common::compose_trs_matrix(
        translate,
        [
            rotate_deg[0].to_radians(),
            rotate_deg[1].to_radians(),
            rotate_deg[2].to_radians(),
        ],
        [1.0, 1.0, 1.0],
        [0.0, 0.0, 0.0],
    );
    let transform = AffineTransform::new(matrix)?;

    // Auto-ground: absent -> grounded iff no component precedes this one.
    let fixed = match ctx.param("isFixed") {
        None | Some(serde_json::Value::Null) => ctx.scene.components.is_empty(),
        Some(serde_json::Value::Bool(flag)) => *flag,
        Some(other) => return Err(format!("param `isFixed` must be a boolean, found {other}")),
    };

    // FAST lane, with the SELF-HEAL fallback (dirty entry or unreadable
    // snapshot -> re-execute the embedded document, rewrite the snapshot) and
    // the STALE lane (a snapshot from another kernel build is rebuilt).
    let stale = !entry.dirty
        && parts_library::kernel_source_stamp().is_some_and(|stamp| entry.snapshot_producer != stamp);
    // A sub-assembly whose constraint rows this session has not seen (a loaded
    // document: the rows are not saved) is rebuilt too, so the parent can say
    // how the sub-assembly's own mates ended — and an old snapshot that baked
    // them unsolved is re-solved.
    let mut note: Option<String> = None;
    let (restored, ports, nested, nested_poses): Decoded = if entry.dirty
        || crate::feature_pipeline::extension::document_uses_extensions(&entry.document)
    {
        heal(&part_name, &entry.document)?
    } else if stale {
        rebuild_stale(&part_name, &entry, Rebuild::Stale, &mut note)?
    } else if parts_library::needs_constraint_rows(&entry) {
        rebuild_stale(&part_name, &entry, Rebuild::ConstraintRows, &mut note)?
    } else {
        match restore_solids(&entry.snapshot) {
            Ok(snapshot) => (
                snapshot,
                entry.ports.clone(),
                entry.nested_constraints.clone().unwrap_or_default(),
                entry.nested_poses.clone(),
            ),
            Err(_) => heal(&part_name, &entry.document)?,
        }
    };

    let members: Vec<(String, crate::BrepSolid)> = restored
        .solids
        .into_iter()
        .map(|solid| (solid.name, solid.solid))
        .collect();
    let source = serde_json::json!({
        "sourceKey": entry.source_key,
        "sourceSignature": entry.source_signature,
    });
    let (mut record, added) =
        create_component(&ctx.id, &part_name, fixed, source, transform, members)?;
    // The part's ports ride along, namespaced and posed like its members.
    let posed_ports = component::attach_component_ports(&mut record, &ports);
    // A sub-assembly's constraint rows, under this instance's namespace: every
    // one rides the record (the parent's assembly report lists them), and one
    // that did not end satisfied is noted here, never dropped.
    record.nested_constraints = nested.iter().map(|row| row.namespaced(&ctx.id)).collect();
    // Where the sub-assembly's own solve put its occurrences (part-local).
    record.nested_poses = nested_poses;
    // A FLEXIBLE instance lends its occurrences to the parent's solve
    // (`assembly::flexible`); rigid is the default.
    match ctx.param("flexible") {
        None | Some(serde_json::Value::Null) | Some(serde_json::Value::Bool(false)) => {}
        Some(serde_json::Value::Bool(true)) => {
            let mut spec = flexible_spec(ctx.param("flexiblePoses"), &entry.document);
            for (path, fixed) in ctx.param("flexibleFixed").and_then(|map| map.as_object()).into_iter().flatten() {
                if let Some(fixed) = fixed.as_bool() {
                    spec.fixed.insert(path.clone(), fixed);
                }
            }
            record.flexible = Some(spec);
        }
        Some(other) => return Err(format!("param `flexible` must be a boolean, found {other}")),
    }
    let nested_notes: Vec<String> = record
        .nested_constraints
        .iter()
        .filter(|row| row.is_failure())
        .map(|row| {
            format!(
                "sub-assembly '{part_name}': constraint {} ({}) is {}: {}",
                row.id, row.constraint_type, row.status, row.message
            )
        })
        .collect();

    // Stamp the part's captured metadata under the namespaced names.
    for (name, mut metadata_record) in restored.metadata {
        if let Some(serde_json::Value::String(parent)) = metadata_record.get_mut("inheritsFrom") {
            *parent = component::namespaced(&ctx.id, parent);
        }
        scene_metadata::set_record(&component::namespaced(&ctx.id, &name), metadata_record);
    }

    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.added = added;
    result.components = vec![record];
    result.notes.extend(note);
    result.notes.extend(nested_notes);
    for (id, posed) in posed_ports {
        ports::publish_port(&mut result, &id, posed)?;
    }
    Ok(result)
}

/// The self-heal lane: re-execute the entry's embedded document in isolation,
/// write the fresh snapshot back into the ACTIVE library entry, and decode it —
/// from here on the caller proceeds exactly as the fast lane.
fn heal(part_name: &str, document: &serde_json::Value) -> Result<Decoded, String> {
    let rebuilt = parts_library::rebuild_part(document).map_err(|error| {
        format!("part '{part_name}': re-executing its document failed: {error}")
    })?;
    let restored = restore_solids(&rebuilt.snapshot)
        .map_err(|error| format!("part '{part_name}': rebuilt snapshot did not decode: {error}"))?;
    parts_library::heal_entry(part_name, &rebuilt);
    Ok((restored, rebuilt.ports, rebuilt.nested_constraints, rebuilt.nested_poses))
}

/// A decoded part: its solids and metadata, its ports, its constraint rows and
/// its solved occurrence poses.
type Decoded = (
    RestoredSnapshot,
    BTreeMap<String, PortRecord>,
    Vec<NestedConstraint>,
    BTreeMap<String, crate::AffineTransform>,
);

/// The STALE lane: rebuild a part whose snapshot another kernel build made.
///
/// On success the rebuilt part is used and the entry is rewritten with this
/// build's stamp; `note` is set only when the rebuild CHANGED the part, since an
/// identical rebuild changes nothing the user can see. On failure the stored
/// snapshot is used when it still decodes, with `note` saying so, and the error
/// is returned only when there is nothing to fall back to.
fn rebuild_stale(
    part_name: &str,
    entry: &parts_library::PartsLibraryEntry,
    why: Rebuild,
    note: &mut Option<String>,
) -> Result<Decoded, String> {
    let stored = restore_solids(&entry.snapshot);
    match heal(part_name, &entry.document) {
        Ok((rebuilt, ports, nested, nested_poses)) => {
            let change = match &stored {
                Ok(stored) => describe_change(stored, &rebuilt),
                Err(_) => None,
            };
            if let Some(change) = change {
                *note = Some(format!(
                    "part '{part_name}' was rebuilt from its document because {}; {change}",
                    why.reason()
                ));
            }
            Ok((rebuilt, ports, nested, nested_poses))
        }
        Err(error) => match stored {
            Ok(stored) => {
                *note = Some(format!(
                    "part '{part_name}': {} and rebuilding it failed, so the stored snapshot \
                     is used ({error})",
                    why.reason()
                ));
                Ok((
                    stored,
                    entry.ports.clone(),
                    entry.nested_constraints.clone().unwrap_or_default(),
                    entry.nested_poses.clone(),
                ))
            }
            Err(_) => Err(error),
        },
    }
}

/// Why [`rebuild_stale`] rebuilds a part whose snapshot still decodes.
#[derive(Clone, Copy)]
enum Rebuild {
    /// The snapshot was made by another kernel build.
    Stale,
    /// The part is an assembly whose constraint statuses this session has not
    /// seen; the rebuild solves them (rows are not saved with the snapshot).
    ConstraintRows,
}

impl Rebuild {
    fn reason(self) -> &'static str {
        match self {
            Rebuild::Stale => "its stored snapshot was made by a different kernel build",
            Rebuild::ConstraintRows => {
                "its sub-assembly constraints are solved inside it and had not been evaluated \
                 in this session"
            }
        }
    }
}

/// How a rebuilt part differs from its stored snapshot, or `None` when it does
/// not: the same solids, in order, with the same face counts and volumes equal
/// to 1e-12 relative.
///
/// A volume that cannot be computed on either side is reported, never skipped —
/// an unmeasured solid must not read as an unchanged one.
fn describe_change(stored: &RestoredSnapshot, rebuilt: &RestoredSnapshot) -> Option<String> {
    if stored.solids.len() != rebuilt.solids.len() {
        return Some(format!(
            "its solid count changed from {} to {}",
            stored.solids.len(),
            rebuilt.solids.len()
        ));
    }
    let mut changes = Vec::new();
    let mut worst: Option<(f64, f64, &str)> = None;
    let mut unmeasured = 0usize;
    for (before, after) in stored.solids.iter().zip(&rebuilt.solids) {
        let faces = |solid: &crate::BrepSolid| solid.shells.iter().map(|shell| shell.faces.len()).sum::<usize>();
        let (faces_before, faces_after) = (faces(&before.solid), faces(&after.solid));
        if before.name != after.name || faces_before != faces_after {
            changes.push(format!(
                "'{}' ({faces_before} faces) became '{}' ({faces_after} faces)",
                before.name, after.name
            ));
        }
        match (
            crate::solid_signed_volume(&before.solid),
            crate::solid_signed_volume(&after.solid),
        ) {
            (Ok(volume_before), Ok(volume_after)) => {
                let delta = volume_after - volume_before;
                let relative = delta.abs() / volume_before.abs().max(f64::MIN_POSITIVE);
                if relative > 1e-12 && worst.map_or(true, |(_, known, _)| relative > known) {
                    worst = Some((delta, relative, after.name.as_str()));
                }
            }
            _ => unmeasured += 1,
        }
    }
    if let Some((delta, relative, name)) = worst {
        changes.push(format!(
            "the largest volume change is {delta:+.6e} ({relative:.1e} relative) on '{name}'"
        ));
    }
    if unmeasured > 0 {
        changes.push(format!("the volume of {unmeasured} solid(s) could not be compared"));
    }
    (!changes.is_empty()).then(|| changes.join("; "))
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered - components are inserted from the parts
/// library, not created from a selection.
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

/// This feature's kernel-owned definition (name + parameter schema), aggregated
/// into the catalogue by `feature_pipeline/schema.rs`. The `partName` field is
/// a plain string for now (the insert flow fills it via `add_part_to_library`;
/// a component-selector widget is a Wave-3 form-engine addition), and the
/// `transform` field type is the nested pose object the form engine binds via
/// its path support (`["transform","translate"]`).
pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type": "ACOMP",
        "shortName": "ACOMP",
        "longName": "Assembly Component",
    "ribbonPath": "Home/Assembly/Assembly Component",
    "commandSize": "Large",
        "displayBuilder": false,
        "inputParamsSchema": {
            "id": {
                "type": "string",
                "default_value": null,
                "hint": "unique identifier for the component instance"
            },
            "partName": {
                "type": "string",
                "default_value": null,
                "hint": "parts-library entry this instance places (filled by the insert flow)"
            },
            "transform": {
                "type": "transform",
                "default_value": { "translate": [0, 0, 0], "rotateEulerDeg": [0, 0, 0] },
                "hint": "rigid instance pose: translate [x,y,z] + intrinsic-XYZ Euler degrees (expression-capable)"
            },
            "isFixed": {
                "type": "boolean",
                "default_value": false,
                "label": "Fixed",
                "hint": "grounded — the solver never moves it; the first component of an empty assembly is auto-fixed when unset"
            },
            "flexible": {
                "type": "boolean",
                "default_value": false,
                "label": "Flexible",
                "hint": "a sub-assembly's own mates solve under this assembly's, per instance (its parts can move); off = one rigid body"
            }
        }
    })
}

/// A `{translate, rotateEulerDeg}` pose param (numbers; anything else reads 0)
/// as a rigid transform — the shape the solver write-back persists.
fn pose_param_transform(value: &serde_json::Value) -> Option<AffineTransform> {
    let vec3 = |key: &str| -> [f64; 3] {
        let mut out = [0.0; 3];
        if let Some(items) = value.get(key).and_then(|v| v.as_array()) {
            for (slot, item) in out.iter_mut().zip(items) {
                *slot = item.as_f64().unwrap_or(0.0);
            }
        }
        out
    };
    let (translate, rotate) = (vec3("translate"), vec3("rotateEulerDeg"));
    let matrix = common::compose_trs_matrix(
        translate,
        [rotate[0].to_radians(), rotate[1].to_radians(), rotate[2].to_radians()],
        [1.0, 1.0, 1.0],
        [0.0, 0.0, 0.0],
    );
    AffineTransform::new(matrix).ok()
}

/// What a flexible instance tells the parent's solve: the child document's own
/// constraints, which of its occurrences it grounds (an `isFixed` ACOMP, the
/// auto-grounded first one, or a Fixed constraint's target), where it authors
/// each occurrence, and this instance's persisted occurrence poses.
fn flexible_spec(seeds: Option<&serde_json::Value>, document: &serde_json::Value) -> component::FlexibleSpec {
    let mut spec = component::FlexibleSpec::default();
    let constraints = document
        .get("assembly")
        .and_then(|assembly| assembly.get("constraints"))
        .and_then(|list| list.as_array())
        .cloned()
        .unwrap_or_default();
    let mut first = true;
    for feature in document.get("features").and_then(|f| f.as_array()).into_iter().flatten() {
        if !feature.get("type").and_then(|t| t.as_str()).is_some_and(|t| matches!(t, "ACOMP" | "ASSEMBLY COMPONENT")) {
            continue;
        }
        let params = feature.get("inputParams").cloned().unwrap_or_default();
        let Some(id) = params.get("id").and_then(|id| id.as_str()) else { continue };
        let grounded = match params.get("isFixed") {
            Some(serde_json::Value::Bool(flag)) => *flag,
            _ => first,
        };
        first = false;
        if grounded {
            spec.grounded.insert(id.to_string());
        }
        if let Some(transform) = params.get("transform").and_then(pose_param_transform) {
            spec.authored.insert(id.to_string(), transform);
        }
        // An occurrence the child itself places flexible lends its own
        // occurrences too, from the child's copy of its document.
        if params.get("flexible") == Some(&serde_json::Value::Bool(true)) {
            let part = params.get("partName").and_then(|name| name.as_str()).unwrap_or_default();
            if let Some(inner) = document.pointer(&format!("/partsLibrary/{}/document", part.replace('~', "~0").replace('/', "~1"))) {
                spec.nested.insert(id.to_string(), flexible_spec(None, inner));
            }
        }
    }
    for constraint in &constraints {
        if constraint.get("type").and_then(|t| t.as_str()) != Some("fixed") {
            continue;
        }
        let elements = constraint.pointer("/inputParams/elements").and_then(|e| e.as_array());
        for element in elements.into_iter().flatten().filter_map(|e| e.as_str()) {
            let owner = element.split(':').next().unwrap_or(element);
            spec.grounded.insert(owner.to_string());
        }
    }
    spec.constraints = constraints;
    if let Some(seeds) = seeds.and_then(|seeds| seeds.as_object()) {
        for (path, pose) in seeds {
            if let Some(transform) = pose_param_transform(pose) {
                spec.seeds.insert(path.clone(), transform);
            }
        }
    }
    spec
}

