//! NESTED occurrence lines: the components inside a sub-assembly, at any depth.
//!
//! A nested line is a real line of the product structure. It selects (its
//! member solids, in every placement the row stands for), it shows its
//! `part.` columns (the part's own `partAttributes`) and its `occurrence.`
//! columns (the placing ACOMP's `inputParams.bom` in the owning sub-assembly's
//! definition), and it takes edits:
//!
//! * a `part.` edit writes the part's definition — every copy with the same
//!   source identity, at every depth — through the same refresh the Work part
//!   uses, so the assembly re-runs once;
//! * an `occurrence.` edit writes the owning sub-assembly's definition. When
//!   that definition is placed more than once the edit would change every one
//!   of them, so the cell is shown read-only with that reason as its tooltip
//!   (never blank).
use super::{FLAGS_KEY, ITEM_KEY, QUANTITY_KEY, VISIBLE_KEY};
use crate::column_tree::{RowAction, RowNode};
use crate::panels::assembly_components::{is_acomp_segment, ChainNode};
use crate::panels::bom_columns::{BomColumn, Scope};
use crate::panels::component_actions::ComponentAction;
use brep_render::engine_state::EngineState;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

/// Row menu: make this occurrence the Work part, in place.
pub(crate) const MAKE_WORK_PART: &str = "make-work-part";
/// Row menu: edit this occurrence's part in a tab of its own.
pub(crate) const OPEN_NEW_TAB: &str = "open-new-tab";
/// A file-less part's "Save part as…": its new part file, named by the user
/// (`FileDialog::edit_component_part`).
pub(crate) const SAVE_PART_AS: &str = "save-part-as";

/// What a nested line edits through.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct NestedLine {
    /// Occurrence paths (`ACOMP1:ACOMP2`), one per top-level placement the
    /// row stands for (more than one in the packed view).
    pub paths: Vec<String>,
    pub part_name: String,
    /// Part-name chain from the top document down to this part.
    pub part_names: Vec<String>,
    pub part_source: Option<String>,
    /// Part-name chain down to the OWNING sub-assembly definition.
    pub owner_names: Vec<String>,
    pub owner_source: Option<String>,
    /// The placing ACOMP's id inside the owner.
    pub feature_id: String,
    /// How many placements of the owning definition the assembly holds.
    pub owner_uses: usize,
}

/// What the nested rows are built from.
pub(crate) struct NestedCtx<'a> {
    pub columns: &'a [BomColumn],
    /// Selected entity names (solids, faces, edges).
    pub selected: &'a HashSet<String>,
    /// Every scene solid name with its visibility.
    pub solids: &'a [(String, bool)],
    /// Whether every solid under an occurrence path is visible, by path
    /// ([`visibility_by_prefix`]): a line's visibility cell is a lookup, not a
    /// scan of every solid (550 lines × 500 solids was ~300 ms a frame).
    pub visible_under: &'a HashMap<String, bool>,
    /// The nested lines the user opened (`BomPanel::opened`); the rest draw folded.
    pub opened: &'a HashSet<String>,
    /// The assembly's own placements' part names (use counts).
    pub root_parts: &'a [String],
    /// The assembly's parts library.
    pub library: &'a Value,
    pub lines: &'a mut HashMap<String, NestedLine>,
    /// Every definition's placement count, from ONE walk of the tree
    /// ([`UseCounts`]): a walk per sub-assembly line (56 of them at 500 parts,
    /// ~190 ms a frame) was most of the BOM's frame.
    pub uses: UseCounts,
    /// Each top-level placement's `inputParams`, by id: a nested line's
    /// Fix/Float is decided on the instance that lends it.
    pub top_params: &'a HashMap<String, Value>,
}

/// For every occurrence path that owns a solid (`ACOMP1`, `ACOMP1:ACOMP2`,
/// … — each `:`-prefix of a solid name), whether all of its solids are
/// visible. One pass over the solids, so a line's cell is O(1).
pub(crate) fn visibility_by_prefix(solids: &[(String, bool)]) -> HashMap<String, bool> {
    let mut map: HashMap<String, bool> = HashMap::new();
    for (name, visible) in solids {
        for (at, _) in name.match_indices(':') {
            let all = map.entry(name[..at].to_string()).or_insert(true);
            *all &= *visible;
        }
    }
    map
}

fn source_of(entry: &Value) -> Option<String> {
    entry["sourceKey"].as_str().filter(|s| !s.is_empty()).map(str::to_string)
}

fn acomp<'a>(document: &'a Value, id: &str) -> Option<&'a Value> {
    document["features"].as_array()?.iter().find(|feature| {
        matches!(feature["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT"))
            && feature["inputParams"]["id"].as_str() == Some(id)
    })
}

/// The definition at the end of a part-name chain, from the top library.
pub(crate) fn definition<'a>(library: &'a Value, names: &[String]) -> Option<&'a Value> {
    let (first, rest) = names.split_first()?;
    let mut document = &library[first]["document"];
    for name in rest {
        document = &document["partsLibrary"][name]["document"];
    }
    document.is_object().then_some(document)
}

/// Placements of a definition anywhere in the assembly: every placement whose
/// entry carries `source`, or (embedded-only) whose name chain is `names`.
/// The reference [`UseCounts`] answers from one walk (its test checks them
/// equal); kept for that.
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn count_uses(root_parts: &[String], library: &Value, names: &[String], source: Option<&str>) -> usize {
    fn walk<'a>(
        parts: &mut dyn Iterator<Item = &'a str>,
        library: &'a Value,
        chain: &mut Vec<String>,
        names: &[String],
        source: Option<&str>,
    ) -> usize {
        let mut count = 0;
        for name in parts {
            let entry = &library[name];
            chain.push(name.to_string());
            let same = match source {
                Some(source) => entry["sourceKey"].as_str() == Some(source),
                None => chain.as_slice() == names,
            };
            if same {
                count += 1;
            } else if chain.len() < 32 {
                let document = &entry["document"];
                let mut inner = document["features"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter(|f| matches!(f["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT")))
                    .filter_map(|f| f["inputParams"]["partName"].as_str());
                count += walk(&mut inner, &document["partsLibrary"], chain, names, source);
            }
            chain.pop();
        }
        count
    }
    let mut parts = root_parts.iter().map(String::as_str);
    walk(&mut parts, library, &mut Vec::new(), names, source)
}

/// [`count_uses`] for every definition at once: one walk of the tree counts
/// each placement under its source key (unless a placement of the same
/// source encloses it, where `count_uses` stops descending) and under its
/// name chain (an embedded-only definition's identity).
pub(crate) struct UseCounts {
    by_source: HashMap<String, usize>,
    by_chain: HashMap<Vec<String>, usize>,
}

impl UseCounts {
    pub(crate) fn new(root_parts: &[String], library: &Value) -> Self {
        fn walk<'a>(
            parts: &mut dyn Iterator<Item = &'a str>,
            library: &'a Value,
            chain: &mut Vec<String>,
            sources: &mut Vec<&'a str>,
            counts: &mut UseCounts,
        ) {
            for name in parts {
                let entry = &library[name];
                chain.push(name.to_string());
                *counts.by_chain.entry(chain.clone()).or_default() += 1;
                let source = entry["sourceKey"].as_str();
                if let Some(source) = source {
                    if !sources.contains(&source) {
                        *counts.by_source.entry(source.to_string()).or_default() += 1;
                    }
                }
                if chain.len() < 32 {
                    let document = &entry["document"];
                    let mut inner = document["features"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter(|f| matches!(f["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT")))
                        .filter_map(|f| f["inputParams"]["partName"].as_str());
                    sources.extend(source);
                    walk(&mut inner, &document["partsLibrary"], chain, sources, counts);
                    if source.is_some() {
                        sources.pop();
                    }
                }
                chain.pop();
            }
        }
        let mut counts = UseCounts { by_source: HashMap::new(), by_chain: HashMap::new() };
        let mut parts = root_parts.iter().map(String::as_str);
        walk(&mut parts, library, &mut Vec::new(), &mut Vec::new(), &mut counts);
        counts
    }

    /// What [`count_uses`] answers for `names` / `source`.
    pub(crate) fn get(&self, names: &[String], source: Option<&str>) -> usize {
        match source {
            Some(source) => self.by_source.get(source).copied().unwrap_or(0),
            None => self.by_chain.get(names).copied().unwrap_or(0),
        }
    }
}

/// The child rows of one line, to full depth. `paths` are the line's
/// occurrence paths; `names`/`source`/`document` its definition.
///
/// Only `ACOMP<n>` nodes appear. A BOM lists PARTS and the sub-assemblies a
/// part contains — the bodies inside a part are that part's internals and live
/// on the Scene tree, not here. Filtering recursively also means a part whose
/// chain holds nothing but bodies ends up with no children at all, so the
/// widget draws no collapse box on a row with nothing behind it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn rows(
    ctx: &mut NestedCtx<'_>,
    row_key: &str,
    paths: &[String],
    names: &[String],
    source: Option<&str>,
    nodes: &[ChainNode],
    document: &Value,
) -> Vec<RowNode> {
    let owner_uses = ctx.uses.get(names, source);
    nodes
        .iter()
        .filter(|node| is_acomp_segment(&node.label))
        .map(|node| {
            let id = format!("{row_key}:{}", node.label);
            let child_paths: Vec<String> = paths.iter().map(|p| format!("{p}:{}", node.label)).collect();
            let feature = acomp(document, &node.label);
            let part_name = feature
                .and_then(|f| f["inputParams"]["partName"].as_str())
                .filter(|name| !name.is_empty())
                .map(str::to_string);
            let entry = part_name.as_ref().map(|name| &document["partsLibrary"][name]).unwrap_or(&Value::Null);
            let child_document = &entry["document"];
            let mut child_names = names.to_vec();
            child_names.push(part_name.clone().unwrap_or_default());
            let child_source = source_of(entry);
            let line = NestedLine {
                paths: child_paths.clone(),
                part_name: part_name.clone().unwrap_or_else(|| node.label.clone()),
                part_names: child_names.clone(),
                part_source: child_source.clone(),
                owner_names: names.to_vec(),
                owner_source: source.map(str::to_string),
                feature_id: node.label.clone(),
                owner_uses,
            };
            let mut cells = HashMap::new();
            cells.insert(ITEM_KEY.to_string(), Value::String(line.part_name.clone()));
            cells.insert(QUANTITY_KEY.to_string(), Value::from(child_paths.len() as u64));
            let visible = child_paths.iter().all(|p| ctx.visible_under.get(p).copied().unwrap_or(true));
            cells.insert(VISIBLE_KEY.to_string(), Value::Bool(visible));
            // Fix/Float, per occurrence: one placement per line, decided on
            // the top instance that lends it (`nested_fixed_state`).
            let fixed_states: Vec<Result<bool, String>> = child_paths
                .iter()
                .map(|path| {
                    let (top, rest) = path.split_once(':').unwrap_or((path, ""));
                    let params = ctx.top_params.get(top).unwrap_or(&Value::Null);
                    let part = params["partName"].as_str().unwrap_or_default();
                    brep_render::engine_state::nested_fixed_state(params, rest, &ctx.library[part]["document"])
                })
                .collect();
            let fixed = !fixed_states.is_empty() && fixed_states.iter().all(|state| state == &Ok(true));
            let mut flags = Vec::new();
            if fixed {
                flags.push(serde_json::json!({
                    "glyph": crate::panels::assembly_components::FIXED_GLYPH,
                    "tooltip": "Grounded — unfix it before moving it",
                }));
            }
            cells.insert(FLAGS_KEY.to_string(), Value::Array(flags));
            let occurrence = feature.map(|f| &f["inputParams"]["bom"]).unwrap_or(&Value::Null);
            let part_attributes = &child_document[brep_render::engine_state::PART_ATTRIBUTES];
            let mut locked = HashMap::new();
            let plm_part = child_source
                .as_deref()
                .is_some_and(|key| crate::plm::bom::revision_of_document(key).is_some());
            for column in ctx.columns {
                let key = column.key();
                if key == QUANTITY_KEY {
                    continue;
                }
                let value = match column.scope {
                    Scope::Part => &part_attributes[&column.field],
                    Scope::Occurrence => &occurrence[&column.field],
                };
                if !value.is_null() {
                    cells.insert(key.clone(), value.clone());
                }
                match column.scope {
                    Scope::Occurrence if owner_uses > 1 => {
                        locked.insert(
                            key,
                            format!(
                                "Stored on this placement inside '{}', which is placed {owner_uses} times; edit it in '{}' to change every placement",
                                names.last().map(String::as_str).unwrap_or(""),
                                names.last().map(String::as_str).unwrap_or(""),
                            ),
                        );
                    }
                    // A PLM part's other attributes are edited here like a
                    // top-level line's: a PATCH, applied once the server takes it.
                    Scope::Part if plm_part && super::PLM_PART_FIELDS.iter().any(|f| column.field.eq_ignore_ascii_case(f)) => {
                        locked.insert(key, "The PLM's own: it is not edited here".to_string());
                    }
                    _ => {}
                }
            }
            let selected = is_selected(ctx, &child_paths);
            let mut actions = vec![
                RowAction::new(MAKE_WORK_PART, "Make Work Part")
                    .tooltip("Edit this part here, in the context of the assembly (double click)"),
                RowAction::new(OPEN_NEW_TAB, "Open in new tab")
                    .tooltip("Edit this part in a tab of its own; the assembly updates as you edit"),
            ];
            if child_source.as_deref().is_none_or(str::is_empty) {
                actions.push(
                    RowAction::new(SAVE_PART_AS, "Save part as\u{2026}")
                        .tooltip("This part has no file of its own: save it to a new part file you name"),
                );
            }
            let toggle = ComponentAction::ToggleFixed;
            let entry = RowAction::new(toggle.id(), toggle.label(fixed)).tooltip(toggle.tooltip()).separator_above();
            actions.push(match fixed_states.as_slice() {
                [Ok(_)] => entry,
                [Err(reason)] => entry.disabled(reason.clone()),
                many => entry.disabled(format!("{} placements on this row — switch to Unpacked to fix or unfix one", many.len())),
            });
            let children = rows(
                ctx,
                &id,
                &child_paths,
                &child_names,
                child_source.as_deref(),
                &node.children,
                child_document,
            );
            let resolvable = part_name.is_some() && child_document.is_object();
            ctx.lines.insert(id.clone(), line);
            let expanded = ctx.opened.contains(&id);
            RowNode {
                children,
                id,
                cells,
                editable: resolvable,
                selected,
                expanded,
                actions: if resolvable { actions } else { Vec::new() },
                locked,
            }
        })
        .collect()
}

/// A nested line reads selected when the selection reaches into it and its
/// parent occurrence is not selected as a whole (then the parent is the
/// selection, not this line).
fn is_selected(ctx: &NestedCtx<'_>, paths: &[String]) -> bool {
    paths.iter().any(|path| {
        let prefix = format!("{path}:");
        if !ctx.selected.iter().any(|name| name.starts_with(&prefix)) {
            return false;
        }
        let parent = match path.rsplit_once(':') {
            Some((parent, _)) => format!("{parent}:"),
            None => return true,
        };
        let mut parent_solids = ctx.solids.iter().filter(|(name, _)| name.starts_with(&parent)).peekable();
        parent_solids.peek().is_some()
            && !parent_solids.all(|(name, _)| ctx.selected.contains(name))
    })
}

/// Select the member solids of these occurrence paths; `additive` toggles
/// them into the current selection (Ctrl/Shift click).
pub(crate) fn select_paths(state: &mut EngineState, paths: &[String], additive: bool) {
    let members: Vec<String> = state
        .scene
        .solids()
        .iter()
        .filter(|solid| paths.iter().any(|p| solid.name.starts_with(&format!("{p}:"))))
        .map(|solid| solid.name.clone())
        .collect();
    let mut solids: Vec<String> = if additive {
        state.emphasis.selected_solids.iter().cloned().collect()
    } else {
        Vec::new()
    };
    let all_in = !members.is_empty() && members.iter().all(|m| solids.contains(m));
    if additive && all_in {
        solids.retain(|s| !members.contains(s));
    } else {
        for member in members {
            if !solids.contains(&member) {
                solids.push(member);
            }
        }
    }
    let json = serde_json::json!({ "selected": { "solids": solids } }).to_string();
    let _ = state.emphasis.apply_json(&json);
    state.dirty = true;
}

/// Write one attribute into `record_key` of `owner` (`null`/`""` removes it;
/// an emptied record goes too, so an untouched document serializes as before).
pub(crate) fn write_record(owner: &mut Value, record_key: &str, key: &str, value: Value) {
    let remove = value.is_null() || value.as_str().is_some_and(str::is_empty);
    let Some(object) = owner.as_object_mut() else { return };
    if remove {
        if let Some(record) = object.get_mut(record_key).and_then(Value::as_object_mut) {
            record.remove(key);
            if record.is_empty() {
                object.remove(record_key);
            }
        }
        return;
    }
    let record = object
        .entry(record_key.to_string())
        .or_insert_with(|| Value::Object(Default::default()));
    if !record.is_object() {
        *record = Value::Object(Default::default());
    }
    record[key] = value;
}

/// Replace the definition at `names` (and every copy with `source`) in the
/// assembly with `document`, as one undo step.
pub(crate) fn apply_definition(
    state: &mut EngineState,
    names: &[String],
    source: Option<&str>,
    document: &Value,
) -> Result<(), String> {
    let mut assembly: Value =
        serde_json::from_str(&state.history_request_json()).map_err(|e| e.to_string())?;
    let mut notes = Vec::new();
    if !crate::document::work_part::refresh_parts(&mut assembly, names, source, document, &mut notes)? {
        return Err("the assembly no longer holds this definition".into());
    }
    state.edit_document_json(&assembly.to_string())?;
    for note in notes {
        state.push_notice(format!("BOM: {note}"));
    }
    Ok(())
}

/// Write one part attribute into the definition at `names` and every copy
/// of it with `source`, as one undo step.
pub(crate) fn set_part_attribute(
    state: &mut EngineState,
    names: &[String],
    source: Option<&str>,
    field: &str,
    value: Value,
) -> Result<(), String> {
    let mut document = definition(state.history.parts_library(), names)
        .cloned()
        .ok_or("the part's definition is missing")?;
    write_record(&mut document, brep_render::engine_state::PART_ATTRIBUTES, field, value);
    apply_definition(state, names, source, &document)
}

/// Apply one cell edit to a nested line.
pub(crate) fn apply_edit(
    state: &mut EngineState,
    line: &NestedLine,
    column: &BomColumn,
    value: Value,
) -> Result<(), String> {
    match column.scope {
        Scope::Part => set_part_attribute(state, &line.part_names, line.part_source.as_deref(), &column.field, value),
        Scope::Occurrence => {
            if line.owner_uses > 1 {
                return Err(format!(
                    "'{}' is placed {} times; edit the placement in its own document",
                    line.owner_names.last().map(String::as_str).unwrap_or(""),
                    line.owner_uses
                ));
            }
            let mut document = definition(state.history.parts_library(), &line.owner_names)
                .cloned()
                .ok_or("the sub-assembly's definition is missing")?;
            let feature = document["features"]
                .as_array_mut()
                .and_then(|features| {
                    features.iter_mut().find(|f| f["inputParams"]["id"].as_str() == Some(&line.feature_id))
                })
                .ok_or("the placement is missing from its sub-assembly")?;
            write_record(
                &mut feature["inputParams"],
                brep_render::engine_state::OCCURRENCE_ATTRIBUTES,
                &column.field,
                value,
            );
            apply_definition(state, &line.owner_names, line.owner_source.as_deref(), &document)
        }
    }
}

/// The state the BOM publishes for one nested line.
pub(crate) fn line_state(row: &RowNode, line: &NestedLine) -> Value {
    serde_json::json!({
        "partName": line.part_name,
        "paths": line.paths,
        "selected": row.selected,
        "editable": row.editable,
        "cells": row.cells,
        "locked": row.locked,
        "ownerUses": line.owner_uses,
    })
}
