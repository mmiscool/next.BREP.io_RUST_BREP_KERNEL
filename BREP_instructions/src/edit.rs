//! Step editing: merge, split, insert, reorder, nest, move a part between
//! steps, update a step's fields, and each step's presentation.
//!
//! Merge, split, reorder and a part's move stay among siblings (the steps
//! under one parent): a group builds one thing. [`nest_step`] moves a step to
//! another parent, children and all. Each refusal says why.

use crate::model::{Add, Arrow, Callout, Instructions, Signoff, Step, Torque, ToolUse};
use crate::wizard::{Component, Level};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashSet;

/// Merge `step_ids` (two or more siblings) into the first of them in their
/// order, which keeps its place: adds, tools, callouts, arrows and child
/// steps are appended in order, notes joined, and the first torque and view
/// kept.
pub fn merge(block: &mut Instructions, step_ids: &[&str]) -> Result<String, String> {
    if step_ids.len() < 2 {
        return Err("pick two or more steps to merge".into());
    }
    let mut located = Vec::new();
    for id in step_ids {
        let parent = block.parent_id(id).ok_or_else(|| format!("no step '{id}'"))?.map(str::to_string);
        let (_, index) = block.siblings(id).ok_or_else(|| format!("no step '{id}'"))?;
        if located.contains(&(parent.clone(), index)) {
            return Err(format!("step '{id}' is named twice"));
        }
        located.push((parent, index));
    }
    if located.iter().any(|(parent, _)| *parent != located[0].0) {
        return Err("only sibling steps can be merged (steps under the same parent): these are not siblings".into());
    }
    let mut indices: Vec<usize> = located.iter().map(|(_, i)| *i).collect();
    indices.sort_unstable();
    let (steps, _) = block.siblings_mut(step_ids[0]).ok_or_else(|| format!("no step '{}'", step_ids[0]))?;
    let mut absorbed: Vec<Step> = Vec::new();
    for &i in indices[1..].iter().rev() {
        absorbed.push(steps.remove(i));
    }
    absorbed.reverse();
    let keeper = &mut steps[indices[0]];
    for step in absorbed {
        keeper.adds.extend(step.adds);
        for tool in step.tools {
            if !keeper.tools.contains(&tool) {
                keeper.tools.push(tool);
            }
        }
        keeper.callouts.extend(step.callouts);
        keeper.arrows.extend(step.arrows);
        if !step.notes.is_empty() {
            if !keeper.notes.is_empty() {
                keeper.notes.push('\n');
            }
            keeper.notes.push_str(&step.notes);
        }
        if keeper.torque.is_none() {
            keeper.torque = step.torque;
        }
        if keeper.view.is_none() {
            keeper.view = step.view;
        }
        if keeper.motion.is_none() {
            keeper.motion = step.motion;
        }
        keeper.signoff.required |= step.signoff.required;
        keeper.steps.extend(step.steps);
    }
    keeper.coalesce_adds();
    Ok(keeper.id.clone())
}

/// Split the adds at `add_indices` out of `step_id` into a new step right
/// after it, with the arrows and callouts that point at their occurrences.
/// Returns the new step's id.
pub fn split(block: &mut Instructions, step_id: &str, add_indices: &[usize]) -> Result<String, String> {
    let count = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?.adds.len();
    let picked: HashSet<usize> = add_indices.iter().copied().collect();
    if picked.is_empty() {
        return Err("pick the parts to split into a new step".into());
    }
    if let Some(bad) = picked.iter().find(|&&i| i >= count) {
        return Err(format!("step '{step_id}' has no part {bad}"));
    }
    if picked.len() == count {
        return Err("a split must leave at least one part in the step".into());
    }
    let id = block.next_id("MS");
    let (steps, s) = block.siblings_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let step = &mut steps[s];
    let (moved, kept): (Vec<_>, Vec<_>) =
        std::mem::take(&mut step.adds).into_iter().enumerate().partition(|(i, _)| picked.contains(i));
    step.adds = kept.into_iter().map(|(_, a)| a).collect();
    let moved: Vec<_> = moved.into_iter().map(|(_, a)| a).collect();
    let moved_occurrences: HashSet<&str> = moved.iter().flat_map(|a| a.occurrences.iter().map(String::as_str)).collect();
    let (arrows, kept_arrows): (Vec<Arrow>, Vec<Arrow>) =
        std::mem::take(&mut step.arrows).into_iter().partition(|a| moved_occurrences.contains(a.occurrence.as_str()));
    step.arrows = kept_arrows;
    let (callouts, kept_callouts): (Vec<Callout>, Vec<Callout>) = std::mem::take(&mut step.callouts)
        .into_iter()
        .partition(|k| k.occurrence.as_deref().is_some_and(|o| moved_occurrences.contains(o)));
    step.callouts = kept_callouts;
    let torque = if moved.iter().any(|a| a.fastener) { step.torque.clone() } else { None };
    if torque.is_some() && !step.adds.iter().any(|a| a.fastener) {
        step.torque = None;
    }
    let title = match moved.as_slice() {
        [one] if one.quantity() == 1 => format!("Install {}", one.part),
        [one] => format!("Install {} × {}", one.quantity(), one.part),
        _ => format!("{} (continued)", step.title),
    };
    let tools = step.tools.clone();
    let new = Step { id: id.clone(), title, adds: moved, tools, torque, callouts, arrows, ..Step::default() };
    steps.insert(s + 1, new);
    Ok(id)
}

/// Move `step_id` to `index` among its siblings (clamped to the end).
pub fn move_step(block: &mut Instructions, step_id: &str, index: usize) -> Result<(), String> {
    let (steps, s) = block.siblings_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let step = steps.remove(s);
    let index = index.min(steps.len());
    steps.insert(index, step);
    Ok(())
}

/// Move `step_id`, children and all, under `parent` (`None`: the top level)
/// at `index` among its new siblings (clamped to the end). Its id, and its
/// children's, never change. A step cannot go inside itself.
pub fn nest_step(block: &mut Instructions, step_id: &str, parent: Option<&str>, index: usize) -> Result<(), String> {
    let step = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    if let Some(parent) = parent {
        if step.contains(parent) {
            return Err(format!("step '{step_id}' cannot be nested inside itself"));
        }
        block.step(parent).ok_or_else(|| format!("no step '{parent}'"))?;
    }
    let (siblings, s) = block.siblings_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let step = siblings.remove(s);
    let list = match parent {
        None => &mut block.steps,
        Some(parent) => &mut block.step_mut(parent).ok_or_else(|| format!("no step '{parent}'"))?.steps,
    };
    let index = index.min(list.len());
    list.insert(index, step);
    Ok(())
}

/// Move one add (by index) from `from` to `to`, a sibling step. The source
/// step may be left empty; it is then listed by [`empty_steps`].
pub fn move_add(block: &mut Instructions, from: &str, add_index: usize, to: &str) -> Result<(), String> {
    let from_parent = block.parent_id(from).ok_or_else(|| format!("no step '{from}'"))?.map(str::to_string);
    let to_parent = block.parent_id(to).ok_or_else(|| format!("no step '{to}'"))?.map(str::to_string);
    if from_parent != to_parent {
        return Err("a part can only move to a sibling step (one under the same parent)".into());
    }
    if from == to {
        return Ok(());
    }
    let source = block.step_mut(from).ok_or_else(|| format!("no step '{from}'"))?;
    if add_index >= source.adds.len() {
        return Err(format!("step '{from}' has no part {add_index}"));
    }
    let add = source.adds.remove(add_index);
    let occurrences: HashSet<String> = add.occurrences.iter().cloned().collect();
    let (arrows, kept): (Vec<Arrow>, Vec<Arrow>) =
        std::mem::take(&mut source.arrows).into_iter().partition(|a| occurrences.contains(&a.occurrence));
    source.arrows = kept;
    let target = block.step_mut(to).ok_or_else(|| format!("no step '{to}'"))?;
    target.adds.push(add);
    target.coalesce_adds();
    target.arrows.extend(arrows);
    Ok(())
}

/// A new empty step right after `after` among its siblings, or at the end of
/// the top level. Parts come to it by [`move_add`]. Returns its id.
pub fn insert_step(block: &mut Instructions, after: Option<&str>) -> Result<String, String> {
    if let Some(step_id) = after {
        block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    }
    let id = block.next_id("MS");
    let step = Step::new(id.clone(), "New step".into());
    match after {
        Some(step_id) => {
            let (steps, s) = block.siblings_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
            steps.insert(s + 1, step);
        }
        None => block.steps.push(step),
    }
    Ok(id)
}

/// What [`add_parts`] did: how many occurrences joined the step, and the
/// ones left where they were, each with the step that holds it (or the
/// sub-assembly it is inside).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AddReport {
    pub added: usize,
    #[serde(default)]
    pub left: Vec<LeftOccurrence>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LeftOccurrence {
    pub occurrence: String,
    /// The step that adds it, or the step that places the sub-assembly it
    /// is inside.
    pub step: String,
}

/// The step that holds `occurrence`: the one adding it (`exact`), else the
/// one adding its nearest ancestor (a sub-assembly placed whole).
fn holder<'a>(block: &'a Instructions, occurrence: &str) -> Option<(&'a Step, bool)> {
    let mut best: Option<(&Step, usize)> = None;
    for (_, step) in block.steps() {
        for added in step.adds.iter().flat_map(|a| a.occurrences.iter()) {
            if added == occurrence {
                return Some((step, true));
            }
            if occurrence.starts_with(added.as_str()) && occurrence[added.len()..].starts_with(':') && best.is_none_or(|(_, len)| added.len() > len) {
                best = Some((step, added.len()));
            }
        }
    }
    best.map(|(step, _)| (step, false))
}

/// Add `components` (parts of the assembly, by occurrence) to `step_id`: a
/// BOM row dragged into a step. An occurrence already in a step, or inside
/// a sub-assembly another step places whole, is left where it is and
/// reported, unless the target step is inside that sub-assembly's group
/// (a descendant of the step placing it, not that step itself: the step
/// placing the whole never also adds one of its parts). A fastener brings
/// its torque when
/// the step has none. Rows coalesce as everywhere ([`Step::coalesce_adds`]).
pub fn add_parts(block: &mut Instructions, step_id: &str, components: &[&Component]) -> Result<AddReport, String> {
    block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let mut report = AddReport::default();
    let mut adds: Vec<Add> = Vec::new();
    let mut torque = None;
    for component in components {
        match holder(block, &component.path) {
            Some((step, true)) => report.left.push(LeftOccurrence { occurrence: component.path.clone(), step: step.id.clone() }),
            Some((step, false)) if step.id == step_id || !step.contains(step_id) => {
                report.left.push(LeftOccurrence { occurrence: component.path.clone(), step: step.id.clone() })
            }
            _ => {
                if adds.iter().any(|a| a.occurrences.contains(&component.path)) {
                    continue;
                }
                adds.push(Add {
                    find_number: component.find_number.clone(),
                    subassembly: component.children.is_some(),
                    fastener: component.fastener,
                    serial_tracked: component.serial_tracked,
                    ..Add::named(vec![component.path.clone()], component.part_key(), &component.part)
                });
                if component.fastener && torque.is_none() {
                    torque = component.torque.as_deref().and_then(Torque::from_attribute);
                }
                report.added += 1;
            }
        }
    }
    let step = block.step_mut(step_id).expect("checked above");
    step.adds.extend(adds);
    if step.torque.is_none() {
        step.torque = torque;
    }
    step.coalesce_adds();
    Ok(report)
}

/// Take one add (by index) out of `step_id`: the part is in no step any
/// more ("not yet planned", [`unplanned`]); its insertion arrows and the
/// callouts pointing at its occurrences go with it. Returns the add.
pub fn remove_add(block: &mut Instructions, step_id: &str, add_index: usize) -> Result<Add, String> {
    let step = block.step_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    if add_index >= step.adds.len() {
        return Err(format!("step '{step_id}' has no part {add_index}"));
    }
    let add = step.adds.remove(add_index);
    let gone: HashSet<&str> = add.occurrences.iter().map(String::as_str).collect();
    step.arrows.retain(|a| !gone.contains(a.occurrence.as_str()));
    step.callouts.retain(|c| !c.occurrence.as_deref().is_some_and(|o| gone.contains(o)));
    if !step.adds.iter().any(|a| a.fastener) && step.torque.as_ref().is_some_and(|t| t.source == crate::model::TorqueSource::Part) {
        step.torque = None;
    }
    Ok(add)
}

/// The parts of `level` not yet in any step, top-most first: a component no
/// step adds, itself or through an ancestor; and, inside a sub-assembly
/// built step by step (the step placing it has child steps), a part no step
/// adds itself (its own step lost it). A sub-assembly's parts are not listed
/// while the sub-assembly itself is unplanned (they come with it) nor while a
/// step places it whole with no child steps (the whole covers them).
pub fn unplanned<'a>(block: &Instructions, level: &'a Level) -> Vec<&'a Component> {
    fn walk<'a>(block: &Instructions, level: &'a Level, out: &mut Vec<&'a Component>) {
        for component in &level.components {
            match holder(block, &component.path) {
                None => out.push(component),
                Some((placing, false)) => {
                    if !placing.steps.is_empty() {
                        out.push(component);
                    }
                }
                Some((_, true)) => {
                    if let Some(children) = &component.children {
                        walk(block, children, out);
                    }
                }
            }
        }
    }
    let mut out = Vec::new();
    walk(block, level, &mut out);
    out
}

/// Steps with nothing to add and no steps under them (a group with parts
/// only in its children is not empty).
pub fn empty_steps(block: &Instructions) -> Vec<String> {
    block.steps().filter(|(_, s)| s.adds.is_empty() && s.steps.is_empty()).map(|(_, s)| s.id.clone()).collect()
}

/// Delete a step. Its parts must have moved elsewhere first, so no part is
/// silently dropped from the instructions, and its child steps too.
pub fn delete_step(block: &mut Instructions, step_id: &str) -> Result<(), String> {
    let step = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    if !step.adds.is_empty() {
        return Err("move this step's parts to another step before deleting it".into());
    }
    if !step.steps.is_empty() {
        return Err("move or delete the steps under this step before deleting it".into());
    }
    let (steps, s) = block.siblings_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    steps.remove(s);
    Ok(())
}

/// Update a step's authored fields from a JSON patch. Keys: `title`, `notes`,
/// `view` (string or null), `tools` (array), `torque` (object or null; a set
/// torque becomes an override), `callouts`, `arrows`, `signoff`, `motion`.
/// Unknown keys are refused. `adds` change only through the edits above.
pub fn update_step(block: &mut Instructions, step_id: &str, patch: &Value) -> Result<(), String> {
    let object = patch.as_object().ok_or("a step update is a JSON object")?;
    let mut ids_needed = 0;
    if let Some(Value::Array(callouts)) = object.get("callouts") {
        ids_needed = callouts.iter().filter(|c| c.get("id").and_then(Value::as_str).is_none_or(str::is_empty)).count();
    }
    let mut fresh: Vec<String> = (0..ids_needed).map(|_| block.next_id("MK")).collect();
    let step = block.step_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let mut next = step.clone();
    for (key, value) in object {
        let parse = |what: &str| format!("step {key}: not a valid {what}");
        match key.as_str() {
            "title" => next.title = value.as_str().ok_or_else(|| parse("text"))?.to_string(),
            "notes" => next.notes = value.as_str().ok_or_else(|| parse("text"))?.to_string(),
            "view" => next.view = value.as_str().filter(|v| !v.is_empty()).map(str::to_string),
            "tools" => next.tools = serde_json::from_value::<Vec<ToolUse>>(value.clone()).map_err(|_| parse("tools list"))?,
            "torque" => {
                next.torque = if value.is_null() {
                    None
                } else {
                    let mut torque: Torque = serde_json::from_value(with_source(value)).map_err(|_| parse("torque"))?;
                    torque.source = crate::model::TorqueSource::Override;
                    Some(torque)
                }
            }
            "callouts" => {
                let mut list = Vec::new();
                for item in value.as_array().ok_or_else(|| parse("callouts list"))? {
                    let mut item = item.clone();
                    if item.get("id").and_then(Value::as_str).is_none_or(str::is_empty) {
                        item["id"] = Value::String(fresh.remove(0));
                    }
                    list.push(serde_json::from_value::<Callout>(item).map_err(|_| parse("callout"))?);
                }
                next.callouts = list;
            }
            "arrows" => next.arrows = serde_json::from_value::<Vec<Arrow>>(value.clone()).map_err(|_| parse("arrows list"))?,
            "signoff" => next.signoff = serde_json::from_value::<Signoff>(value.clone()).map_err(|_| parse("signoff"))?,
            "motion" => next.motion = (!value.is_null()).then(|| value.clone()),
            other => return Err(format!("a step has no field '{other}'")),
        }
    }
    *step = next;
    Ok(())
}

fn with_source(value: &Value) -> Value {
    let mut value = value.clone();
    if let Some(object) = value.as_object_mut() {
        object.entry("source").or_insert(Value::String("override".into()));
    }
    value
}

/// What a step shows: occurrences placed in earlier steps (drawn ghosted),
/// those this step adds (highlighted), and those not placed yet (hidden).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Presentation {
    pub placed: Vec<String>,
    pub adding: Vec<String>,
    pub hidden: Vec<String>,
}

/// The presentation of `step_id` over `occurrences`, the occurrence paths of
/// the drawn solids (a sub-assembly occurrence stands for every path under
/// it). The step adds what it and its descendants add (a parent installs the
/// group its children built); placed is what its earlier siblings, and their
/// descendants, added; a step inside a group sees nothing of the assembly
/// around the group. Everything else is hidden.
pub fn presentation(block: &Instructions, step_id: &str, occurrences: &[String]) -> Result<Presentation, String> {
    let (siblings, s) = block.siblings(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let mut earlier_list = Vec::new();
    for sibling in &siblings[..s] {
        sibling.occurrences_with_descendants(&mut earlier_list);
    }
    let earlier: HashSet<&str> = earlier_list.into_iter().collect();
    let mut now_list = Vec::new();
    siblings[s].occurrences_with_descendants(&mut now_list);
    let now: HashSet<&str> = now_list.into_iter().collect();
    let mut out = Presentation::default();
    for path in occurrences {
        let covered = |set: &HashSet<&str>| {
            set.contains(path.as_str()) || set.iter().any(|p| path.starts_with(p) && path[p.len()..].starts_with(':'))
        };
        if covered(&now) {
            out.adding.push(path.clone());
        } else if covered(&earlier) {
            out.placed.push(path.clone());
        } else {
            out.hidden.push(path.clone());
        }
    }
    Ok(out)
}

