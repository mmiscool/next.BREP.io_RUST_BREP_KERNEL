//! REPLACE COMPONENT, the app side: what a replacement can be (a stored
//! part, or a part the assembly already carries), which occurrences a
//! "Replace all instances" covers, and the one door the menu, the summary and
//! the automation command share. The engine does the swap and the mate
//! classification (`brep_render::engine_state::replace_component`).

use brep_render::engine_state::{document_signature, ComponentInsert, EngineState};
use eframe::egui;

use crate::automation::hit_keys::HitKeyDoc;
use crate::store::{model_display_name, ModelStore};

/// The parts-library name a stored source takes: a PLM revision is named
/// after its part (`sub-b`, from the store's index), not after its key's
/// last segment, which is the revision label (`B`); a file by its name.
fn stored_display_name(store: &dyn ModelStore, name: &str) -> String {
    crate::plm::bom::revision_of_document(name)
        .and_then(|(part, revision)| store.plm_revision(&crate::plm::identity::document_key(&part, &revision)))
        .map(|row| row.part_name)
        .filter(|part_name| !part_name.is_empty())
        .unwrap_or_else(|| model_display_name(name))
}

/// What the occurrence is replaced WITH.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReplaceSource {
    /// A part document in the store, by its identity (as Insert component
    /// names it).
    Stored(String),
    /// A parts-library entry the assembly already carries.
    Library(String),
}

/// Every ACOMP occurrence that shows the same part as `component_id`
/// (itself included), in history order — the "Replace all instances of
/// `<part>`" set.
pub fn instances_of(engine: &EngineState, component_id: &str) -> Vec<String> {
    let Some(part) = engine.component_info(component_id).map(|info| info.part_name) else {
        return Vec::new();
    };
    (0..engine.history_len())
        .filter_map(|index| engine.feature_id_at(index))
        .filter(|id| engine.component_info(id).is_some_and(|info| info.part_name == part))
        .collect()
}

/// A replace edits the assembly, and a Work part edited in context couples its
/// part's undo to the assembly's: a part undo after the replace would undo the
/// REPLACE, and a push meant for the old part would land on whatever the
/// occurrence shows now. So when the active assembly hosts a Work part
/// session, the replace first finishes it (Return to assembly: its edits are
/// kept) and says so. Returns that line, or `None` when no session was open.
pub fn finish_work_part_for_replace(docs: &mut crate::document::Documents) -> Result<Option<String>, String> {
    let Some(path) = docs.work_part_path().map(str::to_string) else { return Ok(None) };
    let hosted_here = docs.work_part_assembly().is_some_and(|host| std::ptr::eq(host, docs.tab_engine()));
    if !hosted_here {
        return Ok(None);
    }
    docs.return_to_assembly()?;
    let line = format!("Replace: the Work part {path} was finished first (its edits are kept); the replace edits the assembly.");
    docs.engine_mut().push_notice_as(brep_render::engine_state::NoticeSeverity::Info, line.clone());
    Ok(Some(line))
}

/// What a Pattern Component makes of replacing `seed`: opened on one of its
/// instances (`instance`: the instance id, its pattern and number), why the
/// chooser shows the seed; else how many instances follow it. `None` when no
/// pattern is seeded by it.
pub fn pattern_line(engine: &EngineState, seed: &str, instance: Option<&(String, String, i64)>) -> Option<String> {
    let mut follow: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    for id in engine.pattern_instance_ids() {
        if let Some((pattern, s, _)) = brep_render::brep_kernel::pattern_component_instance(&id) {
            if s == seed {
                *follow.entry(pattern).or_default() += 1;
            }
        }
    }
    if let Some((id, pattern, n)) = instance {
        let places = follow.get(pattern).copied().unwrap_or(0) + 1;
        return Some(format!(
            "{id} is instance {n} of {pattern} and follows its seed {seed}: replacing {seed} changes all {places} places."
        ));
    }
    (!follow.is_empty()).then(|| {
        let parts: Vec<String> = follow.iter().map(|(pattern, count)| format!("{count} in {pattern}")).collect();
        format!("Its pattern instances follow it ({}).", parts.join(", "))
    })
}

/// How the replace compares identities for LINEAGE: the store's own
/// spelling, and, in a PLM session, every revision of one PLM part as that
/// part (`plm-part:<id>`): replacing revision A with revision C of the same
/// part keeps the mates by name, as a variation of the same part should.
fn lineage_canonicalizer(store: &dyn ModelStore) -> impl Fn(&str) -> String {
    let canonicalize = store.canonicalizer();
    let plm = store.plm_session().is_some();
    move |key: &str| lineage_key(key, plm, &*canonicalize)
}

/// One identity as lineage compares it (see [`lineage_canonicalizer`]).
fn lineage_key(key: &str, plm: bool, canonicalize: &dyn Fn(&str) -> String) -> String {
    match plm.then(|| crate::panels::plm_host::revision_of(key)).flatten() {
        Some((part, _revision)) => format!("plm-part:{part}"),
        None => canonicalize(key),
    }
}

/// Replace the part shown by `component_ids` with `source`. One undo step;
/// the mates' report lands with the run (`EngineState::replace_report`).
/// Returns the new parts-library entry name.
pub fn replace(
    engine: &mut EngineState,
    store: &dyn ModelStore,
    component_ids: &[String],
    source: &ReplaceSource,
) -> Result<String, String> {
    let canonicalize = lineage_canonicalizer(store);
    match source {
        ReplaceSource::Library(part_name) => engine.replace_components(
            component_ids,
            ComponentInsert::Existing { part_name },
            &canonicalize,
        ),
        ReplaceSource::Stored(name) => {
            let contents = match crate::store::read_now(store, name) {
                crate::store::ReadNow::Ready(contents) => contents,
                crate::store::ReadNow::Loading => return Err(format!("'{name}' is still loading; try again")),
                crate::store::ReadNow::Absent => return Err(format!("'{name}' not found")),
            };
            let display = stored_display_name(store, name);
            engine.replace_components(
                component_ids,
                ComponentInsert::New {
                    name: &display,
                    source_key: name,
                    source_signature: &document_signature(&contents),
                    document_json: &contents,
                },
                &canonicalize,
            )
        }
    }
}

// ---------------------------------------------------------------------------
// Inside a sub-assembly: replace from the top
// ---------------------------------------------------------------------------

fn acomp_mut<'a>(document: &'a mut serde_json::Value, id: &str) -> Option<&'a mut serde_json::Value> {
    document["features"].as_array_mut()?.iter_mut().find(|feature| {
        matches!(feature["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT")) && feature["inputParams"]["id"].as_str() == Some(id)
    })
}

/// Replace a part placed INSIDE a sub-assembly, from the top assembly.
/// `path` is the occurrence path (`ACOMP3:ACOMP1`: the top occurrence, then
/// the placement inside its definition, at any depth). The sub-assembly's
/// definition is edited, so every placement of that sub-assembly follows,
/// and a Save writes it into the sub-assembly's own file. With
/// `all_instances`, every placement of the same part in that sub-assembly is
/// replaced. Mates inside the sub-assembly are held (disabled, marked) when
/// the parts share no lineage, exactly as at the top; the top assembly's mates
/// on the replaced part go through the usual classification. One undo step.
pub fn replace_nested(
    engine: &mut EngineState,
    store: &dyn ModelStore,
    path: &str,
    source: &ReplaceSource,
    all_instances: bool,
) -> Result<String, String> {
    use serde_json::Value;
    let segments: Vec<&str> = path.split(':').collect();
    let [top, inner @ ..] = segments.as_slice() else { return Err("replace: empty path".into()) };
    let Some((target, owners)) = inner.split_last() else {
        return Err(format!("replace: '{path}' is not inside a sub-assembly"));
    };
    let library = engine.history.parts_library().clone();
    let top_part = engine
        .component_info(top)
        .ok_or_else(|| format!("replace: '{top}' is not an assembly component"))?
        .part_name;
    // The definition chain down to the sub-assembly that places `target`.
    let mut names = vec![top_part];
    let mut owner = library[&names[0]].clone();
    for id in owners {
        let part = owner["document"]["features"]
            .as_array()
            .into_iter()
            .flatten()
            .find(|f| f["inputParams"]["id"].as_str() == Some(*id))
            .and_then(|f| f["inputParams"]["partName"].as_str())
            .ok_or_else(|| format!("replace: '{id}' is not placed in '{}'", names.last().unwrap()))?
            .to_string();
        owner = owner["document"]["partsLibrary"][&part].clone();
        names.push(part);
    }
    let owner_source = owner["sourceKey"].as_str().filter(|k| !k.is_empty()).map(str::to_string);
    let mut definition = owner["document"].clone();
    let old_part = definition["features"]
        .as_array()
        .into_iter()
        .flatten()
        .find(|f| f["inputParams"]["id"].as_str() == Some(*target))
        .and_then(|f| f["inputParams"]["partName"].as_str())
        .ok_or_else(|| format!("replace: '{target}' is not placed in '{}'", names.last().unwrap()))?
        .to_string();

    // The replacement's entry, as the sub-assembly's library holds it.
    let (display, entry) = match source {
        ReplaceSource::Library(part) => (part.clone(), library[part].clone()),
        ReplaceSource::Stored(name) => {
            let contents = match crate::store::read_now(store, name) {
                crate::store::ReadNow::Ready(contents) => contents,
                crate::store::ReadNow::Loading => return Err(format!("'{name}' is still loading; try again")),
                crate::store::ReadNow::Absent => return Err(format!("'{name}' not found")),
            };
            let document: Value = serde_json::from_str(&contents).map_err(|e| format!("'{name}': {e}"))?;
            let entry = serde_json::json!({
                "sourceKey": name,
                "sourceSignature": document_signature(&contents),
                "document": document,
                "snapshot": "",
            });
            (stored_display_name(store, name), entry)
        }
    };
    if !entry["document"].is_object() {
        return Err("replace: the replacement has no part document".into());
    }
    let mut to = display.clone();
    let mut n = 2;
    while definition["partsLibrary"][&to].is_object() && definition["partsLibrary"][&to]["sourceKey"] != entry["sourceKey"] {
        to = format!("{display} ({n})");
        n += 1;
    }
    if to == old_part {
        return Err(format!("replace: '{path}' already shows '{to}'"));
    }
    let canonicalize = lineage_canonicalizer(store);
    let lineage = brep_render::engine_state::replace_component::entries_lineage(
        &definition["partsLibrary"][&old_part],
        &entry,
        &canonicalize,
    );

    // Which placements: the one, or every one of the same part in there.
    let prefix: String = segments[..segments.len() - 1].join(":");
    let targets: Vec<String> = if all_instances {
        definition["features"]
            .as_array()
            .into_iter()
            .flatten()
            .filter(|f| f["inputParams"]["partName"].as_str() == Some(old_part.as_str()))
            .filter_map(|f| f["inputParams"]["id"].as_str().map(str::to_string))
            .collect()
    } else {
        vec![target.to_string()]
    };
    if !definition["partsLibrary"].is_object() {
        definition["partsLibrary"] = serde_json::json!({});
    }
    definition["partsLibrary"][&to] = entry;
    for id in &targets {
        if let Some(feature) = acomp_mut(&mut definition, id) {
            feature["inputParams"]["partName"] = Value::String(to.clone());
        }
    }
    // The sub-assembly's own mates on a replaced placement: held when the
    // parts are not related (the run inside the sub-assembly would otherwise
    // resolve colliding names), confirmed later in that sub-assembly.
    let mut held = Vec::new();
    if lineage.is_none() {
        for constraint in definition["assembly"]["constraints"].as_array_mut().into_iter().flatten() {
            let refs: Vec<String> = constraint["inputParams"]["elements"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
                .filter(|r| targets.iter().any(|id| *r == id.as_str() || r.starts_with(&format!("{id}:"))))
                .map(str::to_string)
                .collect();
            if refs.is_empty() {
                continue;
            }
            let was_enabled = constraint["enabled"].as_bool().unwrap_or(true);
            constraint["enabled"] = Value::Bool(false);
            if !constraint["persistentData"].is_object() {
                constraint["persistentData"] = serde_json::json!({});
            }
            // Each reference's frame BEFORE the swap (read through the top
            // assembly, in the top occurrence's frame), kept in the marker:
            // the top window chooses and ticks suggestions from it later.
            let namespaced: Vec<String> = refs.iter().map(|r| format!("{prefix}:{r}")).collect();
            let pre = engine.resolve_refs(&namespaced);
            constraint["persistentData"][brep_render::engine_state::replace_component::REPLACE_MARKER] = serde_json::json!({
                "from": old_part,
                "to": to,
                "wasEnabled": was_enabled,
                "refs": refs.iter().zip(pre.iter().chain(std::iter::repeat(&Value::Null))).map(|(r, pre)| serde_json::json!({
                    "ref": r,
                    "reason": "unconfirmed",
                    "suggestion": { "ref": r, "basis": "same-name", "preTicked": false },
                    "pre": pre,
                })).collect::<Vec<_>>(),
            });
            held.push(constraint["inputParams"]["id"].as_str().unwrap_or("").to_string());
        }
    }

    // ONE document edit through the Work part's refresh (every copy of the
    // sub-assembly definition follows, ancestors rebuild).
    let paths: Vec<String> = targets.iter().map(|id| format!("{prefix}:{id}")).collect();
    let capture = engine.capture_replace(&paths);
    let mut assembly: Value = serde_json::from_str(&engine.history_request_json()).map_err(|e| e.to_string())?;
    let mut notes = Vec::new();
    if !crate::document::work_part::refresh_parts(&mut assembly, &names, owner_source.as_deref(), &definition, &mut notes)? {
        return Err("replace: the assembly no longer holds that sub-assembly".into());
    }
    if !held.is_empty() {
        notes.push(format!(
            "{} mate(s) inside '{}' are held until confirmed there: {} (make it the Work part, then Review)",
            held.len(),
            names.last().unwrap(),
            held.join(", ")
        ));
    }
    // A Pattern Component of that sub-assembly seeded by a replaced
    // placement follows it, as at the top: name it, counting the instances
    // this occurrence of the sub-assembly shows.
    let inner_from: std::collections::BTreeMap<String, String> =
        targets.iter().map(|id| (id.clone(), old_part.clone())).collect();
    let mut instances: Vec<String> = engine
        .scene
        .solids()
        .iter()
        .filter_map(|solid| solid.name.strip_prefix(&format!("{prefix}:")))
        .filter_map(|rest| rest.split(':').next())
        .filter(|id| brep_render::brep_kernel::pattern_component_instance(id).is_some())
        .map(str::to_string)
        .collect();
    instances.sort();
    instances.dedup();
    for note in brep_render::engine_state::replace_component::follow_pattern_seeds(&definition, &inner_from, &to, &instances) {
        notes.push(format!("In '{}', {note}", names.last().unwrap()));
    }
    let from = paths.iter().map(|p| (p.clone(), old_part.clone())).collect();
    let lineage = match lineage {
        Some(word) => paths.iter().map(|p| (p.clone(), word)).collect(),
        None => Default::default(),
    };
    engine.commit_replace(capture, assembly, from, to.clone(), lineage, notes)?;
    Ok(to)
}

// ---------------------------------------------------------------------------
// Held mates at every level: the top window lists, repairs and deletes them
// ---------------------------------------------------------------------------

/// Every mate a replace is holding, at every depth, as the top window lists
/// them. A top-level mate is the engine's row. A mate inside a sub-assembly is
/// listed per occurrence path of that sub-assembly: its id is
/// `<occurrence path>:<constraint id>` (`ACOMP1:CONC1`) and its refs are
/// namespaced the same way (`ACOMP1:ACOMP1:Hole_S`), with its suggestion
/// chosen and ticked from the new part's geometry exactly as at the top.
pub fn held_mates(engine: &mut EngineState) -> Vec<serde_json::Value> {
    use serde_json::Value;
    let mut rows = engine.replace_unresolved().as_array().cloned().unwrap_or_default();
    let library = engine.history.parts_library().clone();
    let mut nested = Vec::new();
    for index in 0..engine.history_len() {
        let Some(id) = engine.feature_id_at(index) else { continue };
        let Some(info) = engine.component_info(&id) else { continue };
        collect_held(&library[&info.part_name]["document"], &id, &info.part_name, 0, &mut nested);
    }
    // A mate held inside a definition is ONE mate however many times that
    // definition is placed: listed once, at its first placement (a Repair
    // there rewrites the definition, so every placement takes it). It was
    // listed once per placement: 18 rows for 9 mates on l500.
    let mut listed: std::collections::BTreeSet<(String, String)> = std::collections::BTreeSet::new();
    nested.retain(|(_, definition, entry)| {
        listed.insert((definition.clone(), entry["inputParams"]["id"].as_str().unwrap_or("").to_string()))
    });
    for (prefix, _, entry) in nested {
        let marker = &entry["persistentData"][brep_render::engine_state::replace_component::REPLACE_MARKER];
        let refs: Vec<Value> = marker["refs"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|row| {
                let local = row["ref"].as_str().unwrap_or("");
                let reference = format!("{prefix}:{local}");
                let owner = format!("{prefix}:{}", local.split(':').next().unwrap_or(local));
                let pre = &row["pre"];
                let suggestion = if pre["ok"] == true {
                    let candidates = engine.occurrence_entities(&owner);
                    match brep_render::engine_state::replace_component::classify_ref(
                        &owner,
                        &reference,
                        pre,
                        &candidates,
                        &Default::default(),
                        false,
                    ) {
                        brep_render::engine_state::replace_component::RefOutcome::Unresolved { suggestion: Some(s), .. } => {
                            serde_json::json!({ "ref": s.reference, "basis": s.basis, "preTicked": s.pre_ticked })
                        }
                        _ => Value::Null,
                    }
                } else {
                    Value::Null
                };
                serde_json::json!({ "ref": reference, "reason": row["reason"], "suggestion": suggestion })
            })
            .collect();
        rows.push(serde_json::json!({
            "id": format!("{prefix}:{}", entry["inputParams"]["id"].as_str().unwrap_or("")),
            "type": entry["type"],
            "origin": marker["origin"].as_str().unwrap_or("replace"),
            "from": marker["from"],
            "to": marker["to"],
            "refs": refs,
        }));
    }
    rows
}

/// What the Constraints pane says about held mates, `(count, wording)`, kept
/// in `cache` per history revision. [`held_mates`] clones the whole parts
/// library and walks every nested definition (~5 ms a frame on l500); the
/// count and the wording read only the document, never the suggestions that
/// follow the scene, and every document write moves the revision (undo and
/// redo included).
pub fn held_summary(cache: &mut Option<(u64, usize, &'static str)>, engine: &mut EngineState) -> (usize, &'static str) {
    let revision = engine.history.revision();
    if let Some((at, count, after)) = *cache {
        if at == revision {
            return (count, after);
        }
    }
    let rows = held_mates(engine);
    let after = if rows.iter().all(|row| row["origin"] == "edit") {
        "a part edit"
    } else if rows.iter().all(|row| row["origin"] != "edit") {
        "Replace"
    } else {
        "Replace or a part edit"
    };
    *cache = Some((revision, rows.len(), after));
    (rows.len(), after)
}

/// Walk a sub-assembly definition placed at `prefix` (its part-name chain
/// `definition`, `/`-joined), collecting its marked constraints and those of
/// everything it places, each with the definition it lives in.
fn collect_held(
    document: &serde_json::Value,
    prefix: &str,
    definition: &str,
    depth: usize,
    out: &mut Vec<(String, String, serde_json::Value)>,
) {
    if depth > 32 || !document.is_object() {
        return;
    }
    for entry in document["assembly"]["constraints"].as_array().into_iter().flatten() {
        if entry["persistentData"][brep_render::engine_state::replace_component::REPLACE_MARKER].is_object() {
            out.push((prefix.to_string(), definition.to_string(), entry.clone()));
        }
    }
    for feature in document["features"].as_array().into_iter().flatten() {
        if !matches!(feature["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT")) {
            continue;
        }
        let (Some(id), Some(part)) = (feature["inputParams"]["id"].as_str(), feature["inputParams"]["partName"].as_str()) else {
            continue;
        };
        collect_held(&document["partsLibrary"][part]["document"], &format!("{prefix}:{id}"), &format!("{definition}/{part}"), depth + 1, out);
    }
}

/// The definition chain of the sub-assembly placed at occurrence `path`
/// (`ACOMP3`, `ACOMP3:ACOMP1`): its part names from the top library down, its
/// source key, and its document.
fn definition_at(engine: &EngineState, path: &str) -> Result<(Vec<String>, Option<String>, serde_json::Value), String> {
    let segments: Vec<&str> = path.split(':').collect();
    let library = engine.history.parts_library().clone();
    let top = engine
        .component_info(segments[0])
        .ok_or_else(|| format!("'{}' is not an assembly component", segments[0]))?
        .part_name;
    let mut names = vec![top];
    let mut entry = library[&names[0]].clone();
    for id in &segments[1..] {
        let part = entry["document"]["features"]
            .as_array()
            .into_iter()
            .flatten()
            .find(|f| f["inputParams"]["id"].as_str() == Some(*id))
            .and_then(|f| f["inputParams"]["partName"].as_str())
            .ok_or_else(|| format!("'{id}' is not placed in '{}'", names.last().unwrap()))?
            .to_string();
        entry = entry["document"]["partsLibrary"][&part].clone();
        names.push(part);
    }
    let source = entry["sourceKey"].as_str().filter(|k| !k.is_empty()).map(str::to_string);
    Ok((names, source, entry["document"].clone()))
}

/// Split a held-mate id into (sub-assembly occurrence path, constraint id);
/// `None` for a top-level mate.
fn nested_id(id: &str) -> Option<(&str, &str)> {
    id.rsplit_once(':')
}

/// Apply edits to nested definitions, one sub-assembly at a time, as ONE
/// document edit (one undo step) through the Work part's refresh.
fn edit_definitions(
    engine: &mut EngineState,
    top: &[(String, String, String)],
    edits: std::collections::BTreeMap<String, Vec<Box<dyn FnOnce(&mut serde_json::Value) -> Result<(), String>>>>,
) -> Result<(), String> {
    let mut assembly: serde_json::Value = serde_json::from_str(&engine.history_request_json()).map_err(|e| e.to_string())?;
    // Top-level repairs ride in the same document edit (one undo step).
    if !top.is_empty() {
        repair_block(&mut assembly["assembly"], top, "")?;
    }
    let mut notes = Vec::new();
    for (path, changes) in edits {
        let (names, source, mut definition) = definition_at(engine, &path)?;
        for change in changes {
            change(&mut definition)?;
        }
        if !crate::document::work_part::refresh_parts(&mut assembly, &names, source.as_deref(), &definition, &mut notes)? {
            return Err(format!("the assembly no longer holds the sub-assembly at '{path}'"));
        }
    }
    engine.edit_document_json(&assembly.to_string())?;
    for note in notes {
        engine.push_notice(format!("Replace: {note}"));
    }
    Ok(())
}

/// Repair held references at any depth, as ONE undo step: `(mate id, old ref,
/// new ref)` with the ids and refs [`held_mates`] lists. A nested mate's new
/// ref must lie inside the same sub-assembly. A mate with nothing left held
/// is re-enabled (if it was enabled before) and loses its marker.
pub fn repair(engine: &mut EngineState, repairs: &[(String, String, String)]) -> Result<(), String> {
    let (top, nested): (Vec<_>, Vec<_>) = repairs.iter().cloned().partition(|(id, _, _)| nested_id(id).is_none());
    if nested.is_empty() {
        return engine.replace_repair(&top);
    }
    let mut edits: std::collections::BTreeMap<String, Vec<Box<dyn FnOnce(&mut serde_json::Value) -> Result<(), String>>>> =
        Default::default();
    for (id, old_ref, new_ref) in nested {
        let (path, _) = nested_id(&id).unwrap();
        let path = path.to_string();
        let prefix = format!("{path}:");
        let local = |r: &str| r.strip_prefix(&prefix).map(str::to_string);
        let old_local = local(&old_ref).ok_or_else(|| format!("'{old_ref}' is not inside '{path}'"))?;
        let new_local = local(&new_ref).ok_or_else(|| format!("pick a face inside '{path}' for {id}"))?;
        let constraint = nested_id(&id).unwrap().1.to_string();
        edits.entry(path).or_default().push(Box::new(move |definition: &mut serde_json::Value| {
            repair_block(&mut definition["assembly"], &[(constraint, old_local, new_local)], "")
        }));
    }
    edit_definitions(engine, &top, edits)
}

/// `replace_repair`'s rule on one `assembly` block (also a nested one).
fn repair_block(block: &mut serde_json::Value, repairs: &[(String, String, String)], _prefix: &str) -> Result<(), String> {
    use brep_render::engine_state::replace_component::REPLACE_MARKER;
    for (id, old_ref, new_ref) in repairs {
        let entry = block["constraints"]
            .as_array_mut()
            .into_iter()
            .flatten()
            .find(|entry| entry["inputParams"]["id"] == id.as_str())
            .ok_or_else(|| format!("no constraint '{id}'"))?;
        let marker = &mut entry["persistentData"][REPLACE_MARKER];
        let refs = marker["refs"].as_array_mut().ok_or_else(|| format!("'{id}' has nothing held"))?;
        let before = refs.len();
        refs.retain(|row| row["ref"] != old_ref.as_str());
        if refs.len() == before {
            return Err(format!("'{id}' holds no ref '{old_ref}'"));
        }
        let done = refs.is_empty();
        let was_enabled = marker["wasEnabled"].as_bool().unwrap_or(true);
        for element in entry["inputParams"]["elements"].as_array_mut().into_iter().flatten() {
            if element == old_ref.as_str() {
                *element = serde_json::Value::String(new_ref.clone());
            }
        }
        if done {
            if let Some(data) = entry["persistentData"].as_object_mut() {
                data.remove(REPLACE_MARKER);
            }
            entry["enabled"] = serde_json::Value::Bool(was_enabled);
        }
    }
    Ok(())
}

/// Accept every PRE-TICKED suggestion of the given held mates (every held
/// mate, at every depth, when `ids` is empty) as one undo step.
pub fn accept_suggestions(engine: &mut EngineState, ids: &[String]) -> Result<usize, String> {
    let mut repairs = Vec::new();
    for mate in held_mates(engine) {
        let id = mate["id"].as_str().unwrap_or_default().to_string();
        if !ids.is_empty() && !ids.contains(&id) {
            continue;
        }
        for row in mate["refs"].as_array().into_iter().flatten() {
            if row["suggestion"]["preTicked"] == true {
                repairs.push((
                    id.clone(),
                    row["ref"].as_str().unwrap_or_default().to_string(),
                    row["suggestion"]["ref"].as_str().unwrap_or_default().to_string(),
                ));
            }
        }
    }
    if repairs.is_empty() {
        return Ok(0);
    }
    repair(engine, &repairs)?;
    Ok(repairs.len())
}

/// Delete a held mate at any depth (one undo step).
pub fn delete_held(engine: &mut EngineState, id: &str) -> Result<(), String> {
    let Some((path, constraint)) = nested_id(id) else {
        return engine.assembly_remove_constraint(id);
    };
    let constraint = constraint.to_string();
    let mut edits: std::collections::BTreeMap<String, Vec<Box<dyn FnOnce(&mut serde_json::Value) -> Result<(), String>>>> =
        Default::default();
    let id = id.to_string();
    edits.entry(path.to_string()).or_default().push(Box::new(move |definition: &mut serde_json::Value| {
        let list = definition["assembly"]["constraints"].as_array_mut().ok_or("the sub-assembly has no constraints")?;
        let before = list.len();
        list.retain(|entry| entry["inputParams"]["id"] != constraint.as_str());
        if list.len() == before {
            return Err(format!("no constraint '{id}'"));
        }
        Ok(())
    }));
    edit_definitions(engine, &[], edits)
}

// ---------------------------------------------------------------------------
// The chooser: what the occurrence can be replaced WITH
// ---------------------------------------------------------------------------

/// One offered replacement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Candidate {
    pub source: ReplaceSource,
    /// What the row says: a family member by its row name (part number), a
    /// file by its display name, a library entry by its entry name.
    pub label: String,
    /// The section it sits in: `revisions` (PLM), `family`, `assembly`,
    /// `folder`, `recent`, `browse`.
    pub group: &'static str,
}

impl Candidate {
    /// The row's stable key (widget keys, automation).
    pub fn key(&self) -> String {
        match &self.source {
            ReplaceSource::Stored(name) => format!("file:{name}"),
            ReplaceSource::Library(part) => format!("part:{part}"),
        }
    }
}

/// Section titles, in the order the chooser shows them.
pub const GROUPS: [(&str, &str); 6] = [
    ("revisions", "Other revisions"),
    ("family", "Same family"),
    ("assembly", "In this assembly"),
    ("folder", "Same folder"),
    ("recent", "Recent"),
    ("browse", "All parts"),
];

/// A listed document that can be a component: anything but a family or a
/// template (a store may list bare names with no extension at all).
fn is_part_file(name: &str) -> bool {
    use crate::document_class::DocumentClass;
    !matches!(DocumentClass::of_name(name), Some(DocumentClass::Family | DocumentClass::Template))
}

/// The candidates for replacing `component_id`, best first: other members of
/// its family (when it is one; by row name), the other parts this assembly
/// already carries, the part files beside it (its folder, or the PLM
/// workspace a PLM store lists), recent files. `browse` adds every part file
/// the store lists. The occurrence's own part is never offered, and neither
/// is `exclude` (the assembly being edited: it cannot contain itself).
pub fn candidates(
    engine: &EngineState,
    store: &dyn ModelStore,
    component_id: &str,
    browse: bool,
    exclude: Option<&str>,
) -> Vec<Candidate> {
    let Some(info) = engine.component_info(component_id) else { return Vec::new() };
    let canonicalize = store.canonicalizer();
    let source_key = engine
        .part_source(&info.part_name)
        .map(|(key, _)| key)
        .filter(|key| !key.is_empty());
    let current = source_key.as_deref().map(|key| canonicalize(key));
    let mut seen: std::collections::BTreeSet<String> = current.iter().cloned().collect();
    if let Some(exclude) = exclude {
        seen.insert(canonicalize(exclude));
    }
    let mut out = Vec::new();
    let push_file = |out: &mut Vec<Candidate>, seen: &mut std::collections::BTreeSet<String>, name: String, label: String, group: &'static str| {
        if seen.insert(canonicalize(&name)) {
            out.push(Candidate { source: ReplaceSource::Stored(name), label, group });
        }
    };

    // In a PLM session: the part's OTHER revisions first (newest first), by
    // label and state. The store lists every indexed revision, loaded or not.
    let listing = store.list();
    if store.plm_session().is_some() {
        if let Some((part, revision)) = source_key.as_deref().and_then(crate::panels::plm_host::revision_of) {
            let mut revisions: Vec<(String, String)> = listing
                .iter()
                .filter_map(|name| {
                    let (other_part, other) = crate::panels::plm_host::revision_of(name)?;
                    (other_part == part && other != revision).then(|| {
                        let key = crate::plm::identity::document_key(&other_part, &other);
                        let label = match store.plm_revision(&key) {
                            Some(entry) => format!("Revision {} ({})", entry.revision_label, entry.lifecycle),
                            None => format!("Revision {other}"),
                        };
                        (name.clone(), label)
                    })
                })
                .collect();
            revisions.reverse();
            for (name, label) in revisions {
                push_file(&mut out, &mut seen, name, label, "revisions");
            }
        }
    }

    // Same family: the family's table rows, by part number, beside the
    // family file (members sit beside their family).
    let document = engine.history.parts_library()[&info.part_name]["document"].clone();
    if let (Some(family), Some(key)) = (crate::family_table::family_source(&document), source_key.as_deref()) {
        let family_identity = crate::store::sibling_identity(key, &family.family);
        if let crate::store::ReadNow::Ready(text) = crate::store::read_now(store, &family_identity) {
            let table = crate::family_table::read_table(&text);
            for row in &table.rows {
                if let Some(member) = crate::family_table::member_identity(Some(&family_identity), &row.part_number) {
                    if store.read(&member).is_some() {
                        push_file(&mut out, &mut seen, member, row.part_number.trim().to_string(), "family");
                    }
                }
            }
        } else {
            // The family file is not here (moved, or members copied without
            // it): the members beside this one still say which family they
            // are, by their stamp.
            for name in store.list() {
                if crate::store::sibling_identity(key, &crate::store::file_name_of(&name)) != name {
                    continue;
                }
                let Some(text) = store.read(&name) else { continue };
                let Ok(other) = serde_json::from_str::<serde_json::Value>(&text) else { continue };
                if let Some(stamp) = crate::family_table::family_source(&other).filter(|s| s.family == family.family) {
                    push_file(&mut out, &mut seen, name, stamp.part_number, "family");
                }
            }
        }
    }

    // Parts this assembly already carries.
    let mut library = engine.history.parts_library().as_object().cloned().unwrap_or_default();
    library.remove(&info.part_name);
    for (part, entry) in library {
        if let Some(key) = entry["sourceKey"].as_str().filter(|k| !k.is_empty()) {
            seen.insert(canonicalize(key));
        }
        out.push(Candidate { source: ReplaceSource::Library(part.clone()), label: part, group: "assembly" });
    }

    // Beside it: the same folder (or the whole PLM workspace).
    if let Some(key) = source_key.as_deref() {
        for name in listing.iter().filter(|name| is_part_file(name)) {
            let beside = crate::store::sibling_identity(key, &crate::store::file_name_of(name)) == *name;
            if beside || store.plm_session().is_some() {
                push_file(&mut out, &mut seen, name.clone(), model_display_name(name), "folder");
            }
        }
    }
    for name in crate::panels::file::recent_documents(store).into_iter().filter(|name| is_part_file(name)) {
        let label = model_display_name(&name);
        push_file(&mut out, &mut seen, name, label, "recent");
    }
    if browse {
        for name in listing.into_iter().filter(|name| is_part_file(name)) {
            let label = model_display_name(&name);
            push_file(&mut out, &mut seen, name, label, "browse");
        }
    }
    out
}

/// Whether `candidate` matches what the user typed: its label, its store
/// name, and in a PLM session the part number and name the index holds for
/// it. Case-insensitive; an empty filter matches everything.
pub fn candidate_matches(store: &dyn ModelStore, candidate: &Candidate, filter: &str) -> bool {
    let filter = filter.trim().to_lowercase();
    if filter.is_empty() {
        return true;
    }
    let mut haystack = candidate.label.to_lowercase();
    match &candidate.source {
        ReplaceSource::Stored(name) => {
            haystack.push(' ');
            haystack.push_str(&name.to_lowercase());
            if let Some((part, revision)) = crate::panels::plm_host::revision_of(name) {
                if let Some(entry) = store.plm_revision(&crate::plm::identity::document_key(&part, &revision)) {
                    haystack.push_str(&format!(" {} {}", entry.part_number, entry.part_name).to_lowercase());
                }
            }
        }
        ReplaceSource::Library(part) => {
            haystack.push(' ');
            haystack.push_str(&part.to_lowercase());
        }
    }
    filter.split_whitespace().all(|word| haystack.contains(word))
}

/// The chooser's list for what was typed: the usual sections when nothing
/// is, and EVERY part the store lists (other folders, the whole PLM index)
/// narrowed to the matches once something is, with no Browse click.
pub fn search_candidates(
    engine: &EngineState,
    store: &dyn ModelStore,
    component_id: &str,
    filter: &str,
    exclude: Option<&str>,
) -> Vec<Candidate> {
    if filter.trim().is_empty() {
        return candidates(engine, store, component_id, false, exclude);
    }
    candidates(engine, store, component_id, true, exclude)
        .into_iter()
        .filter(|candidate| candidate_matches(store, candidate, filter))
        .collect()
}

/// What replacing `component_id` with `candidate` will do, BEFORE it is done.
#[derive(Debug, Clone, PartialEq)]
pub struct Preview {
    /// `family` / `derived` (mates keep their faces by name), `None` for an
    /// unrelated part (every mate waits for confirmation).
    pub lineage: Option<&'static str>,
    /// How many mates touch the occurrence.
    pub mates: usize,
    /// The candidate's own document, for its picture (`None` while loading).
    pub document: Option<serde_json::Value>,
}

impl Preview {
    /// The one line the chooser shows.
    pub fn sentence(&self) -> String {
        match (self.lineage, self.mates) {
            (_, 0) => "No mates on this component.".into(),
            (Some("family"), n) => format!("Same family: the {n} mate(s) keep their faces by name where the faces still exist."),
            (Some(_), n) => format!("A variation of this part: the {n} mate(s) keep their faces by name where the faces still exist."),
            (None, n) => format!("An unrelated part: the {n} mate(s) will wait for your confirmation."),
        }
    }
}

/// The preview for `candidate` (lineage against the occurrence's current
/// part, the mates it touches, the candidate's document). `None` for an
/// unknown component.
pub fn preview(engine: &EngineState, store: &dyn ModelStore, component_id: &str, candidate: &Candidate) -> Option<Preview> {
    let info = engine.component_info(component_id)?;
    let library = engine.history.parts_library();
    let old = &library[&info.part_name];
    let new = match &candidate.source {
        ReplaceSource::Library(part) => Some(library[part].clone()),
        ReplaceSource::Stored(name) => match crate::store::read_now(store, name) {
            crate::store::ReadNow::Ready(text) => serde_json::from_str::<serde_json::Value>(&text)
                .ok()
                .map(|document| serde_json::json!({ "sourceKey": name, "document": document })),
            _ => None,
        },
    };
    let canonicalize = lineage_canonicalizer(store);
    let lineage = new
        .as_ref()
        .and_then(|new| brep_render::engine_state::replace_component::entries_lineage(old, new, &canonicalize));
    let prefix = format!("{component_id}:");
    let mates = engine
        .history
        .assembly_block()
        .and_then(|block| block["constraints"].as_array().cloned())
        .unwrap_or_default()
        .iter()
        .filter(|entry| {
            entry["inputParams"]["elements"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(serde_json::Value::as_str)
                .any(|r| r == component_id || r.starts_with(&prefix))
        })
        .count();
    Some(Preview { lineage, mates, document: new.map(|entry| entry["document"].clone()) })
}

// ---------------------------------------------------------------------------
// The dialog: choose, then the summary (kept / unresolved, repair, delete)
// ---------------------------------------------------------------------------

/// The Replace component window. `open` it on an occurrence; it shows the
/// chooser, runs the replace, then the summary of what happened to the mates
/// until the user closes it. Reopened with no target ([`Self::review`]) it is
/// the summary alone, for mates still held from an earlier replace.
#[derive(Default)]
pub struct ReplaceDialog {
    target: Option<String>,
    /// The document being edited (never offered as its own component).
    document: Option<String>,
    part: String,
    instances: Vec<String>,
    all: bool,
    filter: String,
    candidates: Vec<Candidate>,
    chosen: Option<usize>,
    /// After the replace: the summary is up.
    summary: bool,
    /// User's suggestion ticks, `(constraint, ref)` → ticked; absent = the
    /// suggestion's own `preTicked`.
    ticks: std::collections::BTreeMap<(String, String), bool>,
    /// A repair pick in progress: the reference waiting for a viewport pick.
    picking: Option<(String, String)>,
    /// The chosen candidate is still loading (a PLM revision listed by the
    /// index, not read yet): the replace runs the frame it arrives.
    waiting: Option<usize>,
    /// The filter the candidate list was last built for.
    searched: String,
    /// The chosen candidate's preview and picture, for the key it was made for.
    preview: Option<(String, Preview, Option<egui::TextureHandle>)>,
    /// What a Pattern Component makes of this replace (see [`pattern_line`]).
    pattern: Option<String>,
    status: String,
    hits: std::collections::HashMap<String, egui::Rect>,
}

impl ReplaceDialog {
    pub fn is_open(&self) -> bool {
        self.target.is_some() || self.summary
    }

    /// Open the chooser on `component_id`.
    pub fn open(&mut self, engine: &EngineState, store: &dyn ModelStore, component_id: &str, document: Option<&str>) {
        *self = Self::default();
        // A Pattern Component instance shows its seed's part and has no
        // feature to replace: the chooser opens on the seed and says so.
        let (component_id, instance) = match brep_render::brep_kernel::pattern_component_instance(component_id) {
            Some((pattern, seed, n)) => (seed, Some((component_id.to_string(), pattern, n))),
            None => (component_id.to_string(), None),
        };
        let component_id = component_id.as_str();
        self.pattern = pattern_line(engine, component_id, instance.as_ref());
        let Some(info) = engine.component_info(component_id) else { return };
        self.part = info.part_name;
        self.instances = instances_of(engine, component_id);
        self.target = Some(component_id.to_string());
        self.document = document.map(str::to_string);
        self.candidates = candidates(engine, store, component_id, false, document);
    }

    /// Open the summary alone (mates held by an earlier replace).
    pub fn review(&mut self) {
        *self = Self::default();
        self.summary = true;
    }

    pub fn close(&mut self) {
        *self = Self::default();
    }

    /// The shell's Escape: cancel a repair pick in progress, else close the
    /// chooser. `true` when it took the key. The result window (summary)
    /// stays: held mates are not something to dismiss by a stray key.
    pub fn escape(&mut self) -> bool {
        if self.picking.take().is_some() {
            return true;
        }
        if self.target.is_some() {
            self.close();
            return true;
        }
        false
    }

    /// Replace with candidate `index` (the chooser's Replace button, and the
    /// automation door). Opens the summary. A candidate still loading is
    /// asked for and waited on (`waiting`), not refused.
    pub fn apply(&mut self, engine: &mut EngineState, store: &dyn ModelStore, index: usize) -> Result<(), String> {
        let target = self.target.clone().ok_or("no component to replace")?;
        let candidate = self.candidates.get(index).cloned().ok_or("no such candidate")?;
        if let ReplaceSource::Stored(name) = &candidate.source {
            if matches!(crate::store::read_now(store, name), crate::store::ReadNow::Loading) {
                self.waiting = Some(index);
                self.status = format!("Loading {}\u{2026}", candidate.label);
                return Ok(());
            }
        }
        self.waiting = None;
        let ids = if self.all { self.instances.clone() } else { vec![target] };
        replace(engine, store, &ids, &candidate.source)?;
        self.target = None;
        self.summary = true;
        Ok(())
    }

    /// The automation state of the open window.
    pub fn state_json(&self) -> serde_json::Value {
        serde_json::json!({
            "open": self.is_open(),
            "target": self.target,
            "part": self.part,
            "instances": self.instances,
            "all": self.all,
            "summary": self.summary,
            "picking": self.picking.as_ref().map(|(id, r)| serde_json::json!({ "id": id, "ref": r })),
            "status": self.status,
            "pattern": self.pattern,
            "waiting": self.waiting.is_some(),
            "chosen": self.chosen,
            "preview": self.preview.as_ref().map(|(key, preview, picture)| serde_json::json!({
                "key": key,
                "lineage": preview.lineage,
                "mates": preview.mates,
                "sentence": preview.sentence(),
                "picture": picture.is_some(),
                "loading": preview.document.is_none(),
            })),
            "candidates": self.candidates.iter().map(|c| serde_json::json!({
                "key": c.key(), "label": c.label, "group": c.group,
            })).collect::<Vec<_>>(),
        })
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    pub fn show(&mut self, ctx: &egui::Context, engine: &mut EngineState, store: &dyn ModelStore) {
        self.hits.clear();
        if self.is_open() {
            self.window(ctx, engine, store);
        }
        // Published on every frame, a closed window included: closed by
        // Escape, by its x or by a replace, the blob must say so rather than
        // keep the last open frame's state.
        if crate::automation::registry::enabled() {
            crate::automation::registry::publish("__brepReplaceHit", "replace component window widget rects", &self.hits_json());
            crate::automation::registry::publish("__brepReplace", "replace component window state", &self.state_json().to_string());
        }
    }

    fn window(&mut self, ctx: &egui::Context, engine: &mut EngineState, store: &dyn ModelStore) {
        // A chosen candidate that was still loading: replace when it lands.
        if let Some(index) = self.waiting {
            if let Err(error) = self.apply(engine, store, index) {
                self.waiting = None;
                self.status = error;
            }
            if self.waiting.is_some() {
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
            }
        }
        let mut open = true;
        let title = if self.summary { "Replace component: result" } else { "Replace component" };
        egui::Window::new(title)
            .id(egui::Id::new("replace-component"))
            .open(&mut open)
            .resizable(true)
            .default_size([420.0, 460.0])
            .default_pos(super::below_chrome(ctx, 420.0, 60.0))
            .show(ctx, |ui| {
                if self.summary {
                    self.summary_ui(ui, engine);
                } else {
                    self.chooser_ui(ui, engine, store);
                }
            });
        if !open {
            self.close();
        }
    }

    fn chooser_ui(&mut self, ui: &mut egui::Ui, engine: &mut EngineState, store: &dyn ModelStore) {
        let Some(target) = self.target.clone() else { return };
        // The keyboard, read BEFORE the filter field draws: a focused text
        // field swallows Escape and the arrows, and the shell leaves every key
        // to a focused field, so read later nobody would see them. Arrows move
        // through the shown rows, Enter replaces, Escape closes.
        let (down, up, enter, escape) = ui.ctx().input(|i| {
            (
                i.key_pressed(egui::Key::ArrowDown),
                i.key_pressed(egui::Key::ArrowUp),
                i.key_pressed(egui::Key::Enter),
                i.key_pressed(egui::Key::Escape),
            )
        });
        if escape {
            self.target = None;
            return;
        }
        ui.label(egui::RichText::new(format!("Replace {} ({target}) with:", self.part)).strong());
        if let Some(line) = &self.pattern {
            ui.label(line);
        }
        if self.instances.len() > 1 {
            let label = format!("Replace all {} instances of {}", self.instances.len(), self.part);
            let response = ui.checkbox(&mut self.all, label);
            self.hits.insert("replace:all".into(), response.rect);
        }
        ui.horizontal(|ui| {
            ui.label("Filter");
            let response = ui.text_edit_singleline(&mut self.filter);
            self.hits.insert("replace:filter".into(), response.rect);
            let browse = ui.button("Browse\u{2026}").on_hover_text("List every part file in the store");
            self.hits.insert("replace:browse".into(), browse.rect);
            if browse.clicked() {
                self.candidates = candidates(engine, store, &target, true, self.document.as_deref());
                self.chosen = None;
            }
        });
        ui.separator();
        // Typing searches every part the store lists (no Browse click).
        if self.filter.trim() != self.searched {
            self.searched = self.filter.trim().to_string();
            self.candidates = search_candidates(engine, store, &target, &self.searched, self.document.as_deref());
            self.chosen = None;
        }
        let filter = self.filter.trim().to_lowercase();
        let mut double_clicked = None;
        let mut shown: Vec<usize> = Vec::new();
        egui::ScrollArea::vertical().max_height(300.0).auto_shrink([false, true]).show(ui, |ui| {
            if self.candidates.is_empty() {
                ui.weak("No other parts found: use Browse\u{2026}");
            }
            for (group, heading) in GROUPS {
                let rows: Vec<usize> = (0..self.candidates.len())
                    .filter(|&i| self.candidates[i].group == group)
                    .filter(|&i| candidate_matches(store, &self.candidates[i], &filter))
                    .collect();
                if rows.is_empty() {
                    continue;
                }
                let heading = if group == "folder" && store.plm_session().is_some() { "PLM workspace" } else { heading };
                ui.label(egui::RichText::new(heading).weak().small());
                for i in rows {
                    let response = ui.selectable_label(self.chosen == Some(i), &self.candidates[i].label);
                    self.hits.insert(format!("replace:candidate:{}", self.candidates[i].key()), response.rect);
                    self.hits.insert(format!("replace:row:{}", shown.len()), response.rect);
                    shown.push(i);
                    if response.clicked() {
                        self.chosen = Some(i);
                    }
                    if response.double_clicked() {
                        double_clicked = Some(i);
                    }
                }
            }
        });
        if (down || up) && !shown.is_empty() {
            let at = self.chosen.and_then(|c| shown.iter().position(|&i| i == c));
            let next = match (at, down) {
                (None, true) => 0,
                (None, false) => shown.len() - 1,
                (Some(p), true) => (p + 1).min(shown.len() - 1),
                (Some(p), false) => p.saturating_sub(1),
            };
            self.chosen = Some(shown[next]);
        }
        ui.separator();
        if !self.status.is_empty() {
            ui.colored_label(ui.visuals().warn_fg_color, &self.status);
        }
        // The chosen candidate's preview: its picture and what will happen
        // to the mates.
        if let Some(index) = self.chosen {
            let key = self.candidates[index].key();
            if self.preview.as_ref().is_none_or(|(k, p, _)| *k != key || p.document.is_none()) {
                self.preview = preview(engine, store, &target, &self.candidates[index]).map(|preview| {
                    let picture = preview
                        .document
                        .as_ref()
                        .and_then(crate::plm::thumbnail::embedded_image)
                        .map(|image| ui.ctx().load_texture(format!("replace-preview:{key}"), image, egui::TextureOptions::LINEAR));
                    (key, preview, picture)
                });
            }
            if let Some((_, preview, picture)) = &self.preview {
                ui.horizontal(|ui| {
                    match picture {
                        Some(texture) => {
                            let response = ui.add(egui::Image::new(texture).fit_to_exact_size(egui::vec2(96.0, 96.0)));
                            self.hits.insert("replace:preview".into(), response.rect);
                        }
                        None if preview.document.is_none() => {
                            ui.weak("Loading\u{2026}");
                        }
                        None => {
                            ui.weak("(no picture)");
                        }
                    }
                    ui.label(preview.sentence());
                });
            }
        } else {
            self.preview = None;
        }
        // Enter takes the chosen row, or the top hit of what was typed.
        let top_hit = (!filter.is_empty()).then(|| shown.first().copied()).flatten();
        let mut apply = double_clicked.or(if enter { self.chosen.or(top_hit) } else { None });
        ui.horizontal(|ui| {
            let replace = ui.add_enabled(self.chosen.is_some(), egui::Button::new("Replace"));
            self.hits.insert("replace:apply".into(), replace.rect);
            if replace.clicked() {
                apply = self.chosen;
            }
            let cancel = ui.button("Cancel");
            self.hits.insert("replace:cancel".into(), cancel.rect);
            if cancel.clicked() {
                self.target = None;
            }
        });
        if let Some(index) = apply {
            if let Err(error) = self.apply(engine, store, index) {
                self.status = error;
            }
        }
    }

    fn summary_ui(&mut self, ui: &mut egui::Ui, engine: &mut EngineState) {
        let report = engine.replace_report();
        let unresolved = serde_json::Value::Array(held_mates(engine));
        if report.is_null() && unresolved.as_array().is_none_or(Vec::is_empty) {
            if engine.run_pending() {
                ui.label("Solving\u{2026}");
            } else {
                ui.label("Nothing to review.");
            }
        }
        if !report.is_null() && report["origin"] == "edit" {
            ui.label(format!("After editing {}:", report["to"].as_str().unwrap_or("")));
            let kept = report["kept"].as_array().map_or(0, Vec::len);
            ui.label(format!("{kept} mate(s) kept."));
            for moved in report["reattached"].as_array().into_iter().flatten() {
                ui.label(format!(
                    "{}: re-attached {} \u{2192} {} (the one face on the same carrier)",
                    moved["id"].as_str().unwrap_or(""),
                    moved["from"].as_str().unwrap_or(""),
                    moved["to"].as_str().unwrap_or("")
                ));
            }
            for note in report["notes"].as_array().into_iter().flatten() {
                ui.weak(note.as_str().unwrap_or(""));
            }
        } else if !report.is_null() {
            let lineage = report["lineage"].as_object().and_then(|m| m.values().next()).and_then(|v| v.as_str()).unwrap_or("none");
            let words = match lineage {
                "family" => "a member of the same family: mates kept by face name",
                "derived" => "a variation of the same part: mates kept by face name",
                _ => "an unrelated part: every mate waits for your confirmation",
            };
            ui.label(format!("Now showing {} ({words}).", report["to"].as_str().unwrap_or("")));
            let kept = report["kept"].as_array().map_or(0, Vec::len);
            ui.label(format!("{kept} mate(s) kept."));
            for note in report["notes"].as_array().into_iter().flatten() {
                ui.weak(note.as_str().unwrap_or(""));
            }
        }
        let rows = unresolved.as_array().cloned().unwrap_or_default();
        let mut repairs: Vec<(String, String, String)> = Vec::new();
        let mut delete: Option<String> = None;
        if !rows.is_empty() {
            ui.separator();
            ui.label(egui::RichText::new(format!("{} mate(s) need a face (held, not solved):", rows.len())).strong());
        }
        egui::ScrollArea::vertical().max_height(300.0).auto_shrink([false, true]).show(ui, |ui| {
            for mate in &rows {
                let id = mate["id"].as_str().unwrap_or("").to_string();
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("{id} ({})", mate["type"].as_str().unwrap_or(""))).strong());
                    let delete_button = ui.small_button("\u{2716} Delete mate").on_hover_text("Delete this mate");
                    self.hits.insert(format!("replace:delete:{id}"), delete_button.rect);
                    if delete_button.clicked() {
                        delete = Some(id.clone());
                    }
                });
                for row in mate["refs"].as_array().into_iter().flatten() {
                    let reference = row["ref"].as_str().unwrap_or("").to_string();
                    let edited = mate["origin"] == "edit";
                    let reason = match row["reason"].as_str().unwrap_or("") {
                        "missing" if edited => "gone after the edit, and no single face replaces it",
                        "missing" => "not on the new part",
                        "type-changed" => "now a different kind of face",
                        "moved" => "that corner moved on the new part",
                        "no-longer-maps" => "no longer fits this mate",
                        _ => "unrelated part: confirm",
                    };
                    ui.indent(format!("ref-{id}-{reference}"), |ui| {
                        ui.label(format!("{reference}: {reason}"));
                        let row_key = format!("{id}:{reference}");
                        if let Some(suggestion) = row["suggestion"]["ref"].as_str() {
                            let pre = row["suggestion"]["preTicked"].as_bool().unwrap_or(false);
                            let tick = self.ticks.entry((id.clone(), reference.clone())).or_insert(pre);
                            let basis = if row["suggestion"]["basis"] == "same-name" { "same name" } else { "nearest" };
                            let label = format!("Use {suggestion} ({basis}{})", if pre { ", same geometry" } else { "" });
                            let response = ui.checkbox(tick, label);
                            self.hits.insert(format!("replace:tick:{row_key}"), response.rect);
                        }
                        let picking = self.picking.as_ref() == Some(&(id.clone(), reference.clone()));
                        if picking {
                            let selected: Vec<String> = engine
                                .emphasis
                                .selected_faces
                                .iter()
                                .chain(engine.emphasis.selected_edges.iter())
                                .cloned()
                                .collect();
                            // The pick must lie in the same occurrence: the ref
                            // minus its entity name (`ACOMP1:ACOMP1:Hole_S` →
                            // `ACOMP1:ACOMP1`).
                            let owner = reference.rsplit_once(':').map(|(path, _)| path).unwrap_or("").to_string();
                            let usable = selected.len() == 1 && selected[0].starts_with(&format!("{owner}:"));
                            ui.weak("Select the face or edge to use in the viewport.");
                            let use_button = ui.add_enabled(usable, egui::Button::new("Use selection"));
                            self.hits.insert("replace:use-selection".into(), use_button.rect);
                            if use_button.clicked() {
                                repairs.push((id.clone(), reference.clone(), selected[0].clone()));
                                self.picking = None;
                            }
                        } else {
                            let pick = ui.small_button("Pick\u{2026}").on_hover_text("Pick the replacement face in the viewport");
                            self.hits.insert(format!("replace:pick:{row_key}"), pick.rect);
                            if pick.clicked() {
                                self.picking = Some((id.clone(), reference.clone()));
                                engine.clear_selection();
                            }
                        }
                    });
                }
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            let ticked: Vec<(String, String, String)> = rows
                .iter()
                .flat_map(|mate| {
                    let id = mate["id"].as_str().unwrap_or("").to_string();
                    mate["refs"].as_array().cloned().unwrap_or_default().into_iter().filter_map(move |row| {
                        let reference = row["ref"].as_str()?.to_string();
                        let suggestion = row["suggestion"]["ref"].as_str()?.to_string();
                        Some((id.clone(), reference, suggestion))
                    })
                })
                .filter(|(id, reference, _)| self.ticks.get(&(id.clone(), reference.clone())).copied().unwrap_or(false))
                .collect();
            let apply = ui.add_enabled(!ticked.is_empty(), egui::Button::new(format!("Apply ticked ({})", ticked.len())));
            self.hits.insert("replace:apply-ticked".into(), apply.rect);
            if apply.clicked() {
                repairs.extend(ticked);
            }
            let close = ui.button("Close").on_hover_text("Held mates stay disabled and listed; reopen this from the Constraints panel");
            self.hits.insert("replace:close".into(), close.rect);
            if close.clicked() {
                self.summary = false;
            }
        });
        if !self.status.is_empty() {
            ui.colored_label(ui.visuals().warn_fg_color, &self.status);
        }
        if !repairs.is_empty() {
            match repair(engine, &repairs) {
                Ok(()) => self.status.clear(),
                Err(error) => self.status = error,
            }
        }
        if let Some(id) = delete {
            if let Err(error) = delete_held(engine, &id) {
                self.status = error;
            }
        }
    }
}

/// The hit keys this window publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "replace", prefix: "replace:all", meaning: "Replace all instances of the part (checkbox)", command: Some("assembly_replace_component") },
    HitKeyDoc { panel: "replace", prefix: "replace:filter", meaning: "filter the candidate list", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:browse", meaning: "list every part file in the store", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:candidate:", meaning: "a replacement candidate (replace:candidate:file:<name> or replace:candidate:part:<entry>)", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:row:", meaning: "a candidate by its position in the list as shown (replace:row:0 is the first row, under its section heading); the keyboard moves through the same rows (arrows, Enter replaces, Escape closes)", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:preview", meaning: "the chosen candidate's picture (shown when its document carries one)", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:apply", meaning: "replace with the chosen candidate", command: Some("assembly_replace_component") },
    HitKeyDoc { panel: "replace", prefix: "replace:cancel", meaning: "close the chooser without replacing", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:tick:", meaning: "use the suggested face for an unresolved reference (replace:tick:<mate>:<ref>)", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:pick:", meaning: "pick the replacement face in the viewport (replace:pick:<mate>:<ref>)", command: None },
    HitKeyDoc { panel: "replace", prefix: "replace:use-selection", meaning: "use the selected face for the reference being picked", command: Some("assembly_replace_repair") },
    HitKeyDoc { panel: "replace", prefix: "replace:delete:", meaning: "delete an unresolved mate (replace:delete:<mate>)", command: Some("assembly_remove_constraint") },
    HitKeyDoc { panel: "replace", prefix: "replace:apply-ticked", meaning: "apply every ticked suggestion", command: Some("assembly_replace_accept") },
    HitKeyDoc { panel: "replace", prefix: "replace:close", meaning: "close the summary (held mates stay listed)", command: None },
];

