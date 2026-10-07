//! IMPORT3D — Import 3D Model (STEP). Ported from the retired `Import3dModelFeature`.
//!
//! Three headless sources, tried in order:
//! 0. **`inputParams.nativeBrep`** — a native [`crate::snapshot_solids`] payload
//!    (`io/snapshot.rs`): the exact solids of an earlier import, already carrying
//!    their final IMPORT3D names. Tried FIRST and EXCLUSIVE — a feature carrying
//!    it must not also carry `stepText`/`igesText` (an error, not a precedence
//!    puzzle). The solids register VERBATIM under their stored names: re-deriving
//!    them would break every scene-metadata record and every constraint keyed to a
//!    face name. This is the lane a STEP-assembly part document rides on, and the
//!    reason the snapshot container is a DURABLE format, not a cache (§6 there,
//!    and the `io/snapshot.rs` module doc).
//! 1. **`inputParams.stepText`** — the raw ISO-10303-21 document, baked in by the
//!    toolbar Import lane ([`crate`]'s host: `EngineState::import_step_feature`).
//!    Runs the exact Rust STEP importer
//!    ([`crate::import_step_with_appearance`] — the colour-carrying form).
//! 2. **`persistentData.importCache`** (`kind == "step-brep"`) — SUPERSEDED by
//!    `nativeBrep`. The JS-era persisted EXACT BREP records of a previous import
//!    (`serializeBrepSolid` was the serde `BrepSolid` JSON, so records deserialize
//!    directly) — verbose and unversioned. NOTHING in the Rust tree writes it; the
//!    reader stays only so a JS-era saved model still opens. New payloads go in
//!    `nativeBrep`.
//!
//! Name fidelity (contract rule 2, byte-matching `Import3dModelFeature`):
//! - Body names (`importedSolidNames`): a single body keeps the feature name;
//!   several get `{feature}_SOLID_{N}` with N 1-based, zero-padded to
//!   `max(2, digits(count))`.
//! - Faces (`importedBodyFaceNames` / the single-body fallback): a MULTI-body
//!   import names EVERY face `{bodyName}_Face_{f}` positionally (shell/face
//!   order) so faces stay unique across bodies; a SINGLE body prepends the
//!   (unique) feature name — `{feature}_{stepName}`, or `{feature}_Face_{f}` where
//!   the STEP file left the face unnamed — so two separate imports never share a
//!   face name (the cross-solid name-uniqueness guard's collision class). Edge
//!   names re-derive from the face names, so the derived edges are namespaced too.
//!
//! COLOUR rides the same names. What a STEP file's presentation entities assign
//! (`io/step_import/styles.rs`) is stamped here as a `{"color": "#RRGGBB"}`
//! scene-metadata record on the FINAL body/face names — see `io/appearance.rs`
//! for the convention, [`stamp_appearance`] for the write, and
//! [`native_import_payload_with_appearance`] for the lane that seals it into a
//! part payload. The stamp is a NON-overwriting merge, so a colour edited in the
//! caller's info panel survives a history replay.
//!
//! ONE convention, ONE implementation: [`stamp_imported_names`] holds it, and
//! every lane that names imported bodies goes through it — the live import lanes
//! (via [`add_named_bodies`]) and the payload encoder [`native_import_payload`].
//! Names are stamped ONCE, at import, and then frozen in the payload; a later
//! change here does NOT retro-rename already-imported parts, which is the point
//! (a constraint keyed to a face name survives reload).
//!
//! The failed-import base64 retry lane (`persistentData.failedImportFile`) is
//! not yet migrated — it errors loudly rather than silently skipping.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{
    scene_metadata, AddedSolid, FeatureContext, FeatureRefusal, FeatureResult,
};
use crate::{
    import_iges, import_step_bodies, restore_solids, BodyAppearance, BrepSolid, ImportedColor,
    StepBodyError, COLOR_METADATA_KEY,
};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(refusal) => ctx.fail(refusal),
    }
}

/// `FeatureRefusal`, not `String`: the STEP lane's shell-closure gate refuses a
/// body TYPED (`RefusalClass::UnsoundResult { defect: VectorArea }`,
/// `io/step_import/builder/closure.rs`), and the class has to reach
/// `FeatureResult::refusal` for the app to show it as more than text. Every
/// other failure here is still text, and `?` accepts both.
fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let feature_name = if ctx.id.is_empty() {
        "IMPORT3D".to_string()
    } else {
        ctx.id.clone()
    };

    // 0. Native payload — the exact solids of an earlier import, names frozen in.
    //    EXCLUSIVE by design (see the module doc): a document carrying both this
    //    and a text source is malformed, and answering it with a precedence rule
    //    would silently drop one of the two. Keyed on PRESENCE, not on a
    //    successful `as_str`, so a non-string `nativeBrep` errors here instead of
    //    falling through to a different source.
    if let Some(native) = ctx.param("nativeBrep").filter(|value| !value.is_null()) {
        if ctx.param("stepText").is_some() || ctx.param("igesText").is_some() {
            return Err(
                "import3d: `nativeBrep` is exclusive — a feature carrying it must not also carry `stepText`/`igesText`"
                    .into(),
            );
        }
        let payload = native
            .as_str()
            .ok_or("import3d: param `nativeBrep` must be a base64 snapshot string")?;
        return Ok(restore_native_bodies(ctx, payload)?);
    }

    // 1. Fresh STEP text (baked into `stepText` by the toolbar Import lane).
    if let Some(step_text) = ctx.param("stepText").and_then(|v| v.as_str()) {
        if step_text.contains("ISO-10303-21") {
            let imported = import_step_bodies(step_text).map_err(|error| match error {
                StepBodyError::Text(text) => {
                    FeatureRefusal::Text(format!("import3d: STEP import failed: {text}"))
                }
                StepBodyError::Refused(refusal) => FeatureRefusal::Typed(
                    refusal.with_message(|message| format!("import3d: STEP import refused: {message}")),
                ),
            })?;
            if imported.solids.is_empty() {
                return Err("import3d: STEP file contained no importable solids".into());
            }
            let mut result = add_named_bodies(ctx, imported.solids, &imported.appearances, &feature_name);
            // The closure gate's approximations ride on the feature TYPED,
            // re-keyed from the file index to the scene name the body was
            // just given, so the app and the case runner can name the body.
            for (index, mut approximation) in imported.approximations {
                if let Some(added) = result.added.get(index) {
                    approximation.body = added.name.clone();
                    approximation.message =
                        format!("{}: {}", added.name, approximation.message);
                }
                result.approximations.push(approximation);
            }
            return Ok(result);
        }
        return Err(
            "import3d: only STEP (ISO-10303-21) files are supported (STL/3MF mesh import was removed)"
                .into(),
        );
    }

    // 1b. Fresh IGES text (baked into `igesText` by the toolbar Import lane).
    if let Some(iges_text) = ctx.param("igesText").and_then(|v| v.as_str()) {
        let solids = import_iges(iges_text)
            .map_err(|error| format!("import3d: IGES import failed: {error}"))?;
        if solids.is_empty() {
            return Err("import3d: IGES file contained no importable solids".into());
        }
        return Ok(add_named_bodies(ctx, solids, &[], &feature_name));
    }

    // 2. Persisted exact-BREP records from a previous import.
    let cache = ctx.persistent.get("importCache");
    if let Some(cache) = cache {
        let kind = cache.get("kind").and_then(|v| v.as_str()).unwrap_or("");
        if kind == "step-brep" {
            let records = cache
                .get("kernelSolidRecords")
                .and_then(|v| v.as_array())
                .ok_or("import3d: importCache has no kernelSolidRecords")?;
            let mut solids = Vec::with_capacity(records.len());
            for (index, record) in records.iter().enumerate() {
                let solid: BrepSolid = serde_json::from_value(record.clone()).map_err(|error| {
                    format!("import3d: kernel record {index} failed to deserialize: {error}")
                })?;
                solids.push(solid);
            }
            if solids.is_empty() {
                return Err("import3d: importCache is empty".into());
            }
            return Ok(add_named_bodies(ctx, solids, &[], &feature_name));
        }
        return Err(format!(
            "import3d: unsupported importCache kind '{kind}' (only 'step-brep')"
        )
        .into());
    }

    // A failed-import retry payload without a cache is a not-yet-migrated lane.
    if ctx.persistent.get("failedImportFile").is_some() {
        return Err(
            "import3d: the failed-import base64 retry lane is not yet migrated to the Rust pipeline"
                .into(),
        );
    }

    Err(
        "import3d: no model data (no `nativeBrep`/`stepText` param and no `importCache`)".into(),
    )
}

/// Restore a native payload and register its solids VERBATIM — the `nativeBrep`
/// lane. Nothing about the names is re-derived: they were stamped once, when the
/// geometry was first imported, and every scene-metadata record and every
/// constraint attached to a face is keyed to exactly those strings. (This is
/// `component::create_component`'s phase 2 for the same reason — see its module
/// doc: "do not `fix` this by routing members through `register_added`".)
///
/// The payload's captured metadata records are merged back into the scene store
/// UN-namespaced, exactly as they were captured: an IMPORT3D feature is not a
/// component boundary, so there is no instance prefix to apply. The ACOMP lane
/// namespaces on insert if this document is later used as a part.
fn restore_native_bodies(ctx: &FeatureContext, payload: &str) -> Result<FeatureResult, String> {
    let restored = restore_solids(payload)
        .map_err(|error| format!("import3d: native payload did not decode: {error}"))?;
    if restored.solids.is_empty() {
        return Err("import3d: native payload contained no solids".into());
    }
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    for solid in restored.solids {
        result.added.push(register_verbatim(solid.solid, &solid.name));
    }
    // Overwriting: the payload's records ARE the part's metadata. The pipeline's
    // own `sourceFeatureId` seed runs after this feature returns and is
    // NON-overwriting, so a face keeps the producing-feature id it was imported
    // with rather than picking up this feature's.
    for (name, record) in restored.metadata {
        scene_metadata::merge_record(&name, &record, true);
    }
    Ok(result)
}

/// [`common::register_added`] WITHOUT its two name-deriving passes: collect the
/// names the solid already carries and make it scene-resident.
///
/// The container groups are EMPTY, and that is the right answer rather than a
/// stub: a group is the un-keyed alias a PROFILE-SWEPT feature publishes for its
/// per-loop face families (`{cap_base}_START` standing for every
/// `{cap_base}:L{id}_START` — see [`common::register_added_grouped`]). Imported
/// geometry has no sketch loops behind it, so there is no family to alias; its
/// names arrived stamped from outside and are already the final, unique ones.
fn register_verbatim(solid: BrepSolid, name: &str) -> AddedSolid {
    let face_names = common::collect_face_names(&solid);
    let edge_names = common::collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    AddedSolid {
        handle,
        name: name.to_string(),
        face_names,
        edge_names,
        face_groups: Vec::new(),
        edge_groups: Vec::new(),
    }
}

/// Stamp the established naming convention onto the imported bodies, register
/// them, and stamp whatever COLOUR the file carried onto those same names.
///
/// `appearances` is parallel to `solids` (see [`crate::BodyAppearance`]); pass
/// `&[]` from a lane whose format has no colour, which stamps nothing.
fn add_named_bodies(
    ctx: &FeatureContext,
    mut solids: Vec<BrepSolid>,
    appearances: &[BodyAppearance],
    feature_name: &str,
) -> FeatureResult {
    let body_names = stamp_imported_names(&mut solids, feature_name);
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    for (index, (solid, body_name)) in solids.into_iter().zip(body_names).enumerate() {
        // Colour is stamped AFTER `register_added`, which is where the names
        // become final (it dedupes face names before deriving the edge names).
        let added = common::register_added(solid, &body_name);
        if let Some(appearance) = appearances.get(index) {
            stamp_appearance(&added.name, &added.face_names, appearance);
        }
        result.added.push(added);
    }
    result
}

/// Write an imported body's colours into the scene-metadata store, keyed by the
/// FINAL names — the one place the `{"color": "#RRGGBB"}` convention of
/// `io/appearance.rs` is written.
///
/// `face_names` is `(face_id, name)` in shell/face order
/// ([`common::collect_face_names`]), the same order `appearance.faces` is
/// indexed by. A length disagreement means the two walks have drifted, so
/// nothing per-face is stamped rather than colouring the wrong faces — the body
/// colour, which is not positional, still lands.
///
/// Merged NON-overwriting: a history replay re-runs this feature, and a colour
/// the user has since edited in the info panel must survive it.
fn stamp_appearance(body_name: &str, face_names: &[(u64, String)], appearance: &BodyAppearance) {
    if let Some(color) = appearance.body {
        stamp_color(body_name, color);
    }
    if appearance.faces.is_empty() || appearance.faces.len() != face_names.len() {
        return;
    }
    for ((_, face_name), color) in face_names.iter().zip(&appearance.faces) {
        if let Some(color) = color {
            stamp_color(face_name, *color);
        }
    }
}

fn stamp_color(name: &str, color: ImportedColor) {
    let mut record = serde_json::Map::new();
    record.insert(
        COLOR_METADATA_KEY.to_string(),
        serde_json::Value::String(color.to_hex()),
    );
    scene_metadata::merge_record(name, &record, false);
}

/// The naming half of [`add_named_bodies`]: stamp body + face names onto `solids`
/// IN PLACE and return the body name chosen for each, WITHOUT registering
/// anything. Shared by the live import lanes and the payload encoder
/// [`native_import_payload`], so an imported part is named identically however it
/// arrived.
fn stamp_imported_names(solids: &mut [BrepSolid], feature_name: &str) -> Vec<String> {
    let body_names = imported_solid_names(solids.len(), feature_name);
    let multi_body = solids.len() > 1;
    let mut chosen = Vec::with_capacity(solids.len());
    for (index, solid) in solids.iter_mut().enumerate() {
        let body_name = body_names
            .get(index)
            .cloned()
            .unwrap_or_else(|| feature_name.to_string());
        let mut face_index = 0usize;
        for shell in &mut solid.shells {
            for face in &mut shell.faces {
                if multi_body {
                    // Multi-body: EVERY face namespaced under its body.
                    face.name = Some(format!("{body_name}_Face_{face_index}"));
                } else {
                    // Single body: prepend the (unique) feature name so faces never
                    // collide with ANOTHER import's faces — two single-body imports
                    // both stamped bare `Face_N` before, which the cross-solid
                    // name-uniqueness guard flags. Keep any STEP-authored name as the
                    // stem, else the positional `Face_N` fallback. (`body_name` IS the
                    // feature name here — see `imported_solid_names`.) Edge names are
                    // re-derived from these face names downstream, so namespacing the
                    // faces namespaces the derived edges too.
                    let stem = face
                        .name
                        .as_deref()
                        .map(str::trim)
                        .filter(|name| !name.is_empty())
                        .map(str::to_string)
                        .unwrap_or_else(|| format!("Face_{face_index}"));
                    face.name = Some(format!("{body_name}_{stem}"));
                }
                face_index += 1;
            }
        }
        chosen.push(body_name);
    }
    chosen
}

/// `importedSolidNames` port: 1 body keeps the feature name; several get a
/// stable, zero-padded `{feature}_SOLID_0N` suffix.
pub(crate) fn imported_solid_names(count: usize, feature_name: &str) -> Vec<String> {
    if count == 0 {
        return Vec::new();
    }
    if count == 1 {
        return vec![feature_name.to_string()];
    }
    let digits = count.to_string().len().max(2);
    (1..=count)
        .map(|index| format!("{feature_name}_SOLID_{index:0width$}", width = digits))
        .collect()
}

/// Encode `solids` as a native IMPORT3D payload: stamp the names this feature
/// would stamp under `feature_name`, then seal them into the `io/snapshot`
/// container. The result is exactly what the `nativeBrep` source above reads
/// back, so a part is named identically however it arrived — imported live from
/// a text source, or restored from a payload built here.
///
/// GENERIC, not STEP-specific: any producer of finished `BrepSolid`s that wants
/// them to become an IMPORT3D part document uses this (the STEP-assembly import
/// lane is the first caller).
///
/// The two passes after the stamping are `register_added`'s name-FINALIZING half
/// ([`common::ensure_unique_face_names`] + [`common::stamp_derived_edge_names`]),
/// applied HERE rather than at restore time: the `nativeBrep` lane registers
/// verbatim, so whatever the payload carries IS the final name set. Without them
/// a payload's edge names would be the importer's raw ones while the `stepText`
/// lane's are the derived `{faceA}|{faceB}[n]` — the same geometry under two
/// different name sets, which is exactly what this helper exists to prevent.
///
/// The payload is byte-deterministic for identical input (`snapshot_solids`), so
/// two encodings of the same part collapse to ONE parts-library entry.
pub fn native_import_payload(feature_name: &str, solids: &[BrepSolid]) -> Result<String, String> {
    native_import_payload_with_appearance(feature_name, solids, &[])
}

/// [`native_import_payload`] carrying the import's COLOURS into the payload.
///
/// `appearances` is parallel to `solids` (see [`crate::BodyAppearance`]); `&[]`
/// is the colourless case and makes this identical to [`native_import_payload`].
///
/// The colours are stamped into the AMBIENT scene-metadata store just before
/// `snapshot_solids`, which is exactly how they get INTO the payload — the
/// snapshot captures each stamped name's own record alongside the geometry. So a
/// caller that runs this inside a `crate::IsolatedSceneMetadata` bracket (the
/// STEP-assembly part-document lane does, and must) gets the colours sealed into
/// the part and discarded from the live document, which is the whole point of
/// that bracket: the part carries its own metadata, the open document is not
/// touched.
pub fn native_import_payload_with_appearance(
    feature_name: &str,
    solids: &[BrepSolid],
    appearances: &[BodyAppearance],
) -> Result<String, String> {
    if solids.is_empty() {
        return Err("import3d: native payload needs at least one solid".into());
    }
    let mut bodies = solids.to_vec();
    let body_names = stamp_imported_names(&mut bodies, feature_name);
    for solid in &mut bodies {
        common::ensure_unique_face_names(solid);
        common::stamp_derived_edge_names(solid);
    }
    // Names are FINAL from here, so this is where colour can be keyed to them.
    for (index, (solid, body_name)) in bodies.iter().zip(&body_names).enumerate() {
        if let Some(appearance) = appearances.get(index) {
            stamp_appearance(body_name, &common::collect_face_names(solid), appearance);
        }
    }
    let named: Vec<(&str, &BrepSolid)> = body_names
        .iter()
        .map(String::as_str)
        .zip(bodies.iter())
        .collect();
    crate::snapshot_solids(&named)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered from a selection (no reference inputs).
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

///
/// Only `id` is exposed: an IMPORT3D feature is created by an import lane that
/// bakes its payload in — the toolbar Import lane
/// ([`EngineState::import_step_feature`] / `import_iges_feature`) writes the raw
/// document text into `inputParams.stepText` / `igesText`, and the native lane
/// ([`native_import_payload`], the STEP-assembly part document) writes
/// `inputParams.nativeBrep`. None of the three is form-editable, so none is
/// exposed. (The former `fileToImport` "file" param was a JS-era vestige: nothing
/// produced or consumed it and no form builder rendered it.)
pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "IMPORT3D",
    "shortName": "IMPORT3D",
    "longName": "Import 3D Model",
    "ribbonPath": "Home/Modeling/Import 3D Model",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the import feature"
        }
    }
})
}

