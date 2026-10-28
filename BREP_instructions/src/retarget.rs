//! Re-targeting to a new revision of the assembly.
//!
//! [`diff`] compares the two revisions' occurrence trees by path: an
//! occurrence only in the new one is ADDED, only in the old one REMOVED, and
//! one whose part or content signature changed is REPLACED. [`apply`] folds the
//! diff into the steps, rerunning the wizard on the changed parts only, and
//! records it as the block's pending `retarget`; [`approve`] makes the new
//! revision the target. Until then release is refused.

use crate::model::{for_each_step_mut, retain_steps, Add, Instructions, OccurrenceChange, Retarget, Step, Target};
use crate::wizard::{self, Component, Level};
use std::collections::{HashMap, HashSet};

/// The changes between two revisions of the target assembly.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Diff {
    pub added: Vec<OccurrenceChange>,
    pub removed: Vec<OccurrenceChange>,
    pub replaced: Vec<OccurrenceChange>,
}

impl Diff {
    pub fn is_empty(&self) -> bool {
        self.added.is_empty() && self.removed.is_empty() && self.replaced.is_empty()
    }
}

/// Compare `old` and `new` by occurrence path. An occurrence under a removed
/// or replaced sub-assembly is reported only through that sub-assembly.
pub fn diff(old: &Level, new: &Level) -> Diff {
    let old_map: HashMap<&str, &Component> = wizard::flatten(old).into_iter().map(|(_, c)| (c.path.as_str(), c)).collect();
    let new_map: HashMap<&str, &Component> = wizard::flatten(new).into_iter().map(|(_, c)| (c.path.as_str(), c)).collect();
    let mut out = Diff::default();
    let mut gone: Vec<&str> = Vec::new();
    for (_, c) in wizard::flatten(old) {
        if under_any(&c.path, &gone) {
            continue;
        }
        match new_map.get(c.path.as_str()) {
            None => {
                out.removed.push(change(c, None));
                gone.push(&c.path);
            }
            Some(n) if n.part != c.part || n.signature != c.signature || n.children.is_some() != c.children.is_some() => {
                // A sub-assembly whose definition changed is compared inside
                // instead, unless it stopped (or started) being one or became
                // another part.
                if n.part == c.part && n.children.is_some() && c.children.is_some() {
                    continue;
                }
                out.replaced.push(change(n, Some(&c.part)));
                gone.push(&c.path);
            }
            Some(_) => {}
        }
    }
    let mut fresh: Vec<&str> = Vec::new();
    for (_, c) in wizard::flatten(new) {
        if under_any(&c.path, &gone) || under_any(&c.path, &fresh) {
            continue;
        }
        if !old_map.contains_key(c.path.as_str()) {
            out.added.push(change(c, None));
            fresh.push(&c.path);
        }
    }
    out
}

fn change(c: &Component, was: Option<&str>) -> OccurrenceChange {
    OccurrenceChange { occurrence: c.path.clone(), part: c.part.clone(), was: was.map(str::to_string) }
}

/// Whether `path` lies strictly under one of `roots`.
fn under_any(path: &str, roots: &[&str]) -> bool {
    roots.iter().any(|r| path.len() > r.len() && path.starts_with(r) && path[r.len()..].starts_with(':'))
}

fn under_or_at(path: &str, root: &str) -> bool {
    path == root || under_any(path, &[root])
}

/// Fold `diff` (from `old` to `new`) into `block` and record it as pending
/// for `to`. Returns the recorded retarget. Refuses while one is pending.
pub fn apply(block: &mut Instructions, new: &Level, diff: &Diff, to: Target) -> Result<Retarget, String> {
    if block.retarget.is_some() {
        return Err("a re-target is already waiting for approval".into());
    }
    let mut touched: Vec<String> = Vec::new();
    let new_map: HashMap<&str, &Component> = wizard::flatten(new).into_iter().map(|(_, c)| (c.path.as_str(), c)).collect();

    // Removed and replaced sub-assemblies lose the steps that built them (the
    // children of the step placing them, descendants and all); a replaced one
    // gets fresh children below.
    let dropped: Vec<&str> = diff.removed.iter().chain(&diff.replaced).map(|c| c.occurrence.as_str()).collect();
    for_each_step_mut(&mut block.steps, &mut |step| {
        let places_dropped = step.adds.iter().any(|a| a.subassembly && a.occurrences.iter().any(|o| dropped.iter().any(|d| under_or_at(o, d))));
        if places_dropped {
            step.steps.clear();
        }
    });

    // Removed: strip the occurrence from its add.
    for removed in &diff.removed {
        for_each_step_mut(&mut block.steps, &mut |step| {
            let before = step.adds.iter().map(Add::quantity).sum::<usize>();
            for add in &mut step.adds {
                add.occurrences.retain(|o| !under_or_at(o, &removed.occurrence));
            }
            step.adds.retain(|a| !a.occurrences.is_empty());
            step.arrows.retain(|a| !under_or_at(&a.occurrence, &removed.occurrence));
            if step.adds.iter().map(Add::quantity).sum::<usize>() != before {
                touch(&step.id, &mut touched);
            }
        });
    }

    // Replaced: the occurrence's add takes the new part; an add shared with
    // other occurrences of the old part splits it out; a replaced
    // sub-assembly's step gets the steps that build the new one.
    for replaced in &diff.replaced {
        let Some(component) = new_map.get(replaced.occurrence.as_str()) else { continue };
        let mut fresh_children = component.children.as_ref().map(|children| wizard::level_steps(block, children));
        for_each_step_mut(&mut block.steps, &mut |step| {
            let Some(index) = step.adds.iter().position(|a| a.occurrences.contains(&replaced.occurrence)) else { return };
            let fresh = add_of(component);
            if step.adds[index].occurrences.len() == 1 {
                step.adds[index] = fresh;
            } else {
                step.adds[index].occurrences.retain(|o| o != &replaced.occurrence);
                step.adds.insert(index + 1, fresh);
            }
            if let Some(children) = fresh_children.take() {
                step.steps = children;
            }
            touch(&step.id, &mut touched);
        });
    }

    // Added: the wizard's step for each, placed after the step that adds a
    // mated neighbour; added fasteners of one part share a step.
    let mut grouped: Vec<(String, Vec<&Component>)> = Vec::new();
    for added in &diff.added {
        let Some(component) = new_map.get(added.occurrence.as_str()) else { continue };
        let level_path = parent_level_path(new, &component.path);
        if component.fastener {
            if let Some((_, group)) = grouped.iter_mut().find(|(l, g)| *l == level_path && g[0].part == component.part) {
                group.push(component);
                continue;
            }
        }
        grouped.push((level_path, vec![component]));
    }
    for (level_path, group) in grouped {
        let Some(level) = wizard::find_level(new, &level_path) else { continue };
        if block.group_steps_mut(&level_path).is_none() {
            continue;
        }
        let mut step = wizard::step_for(block, &group);
        if let Some(children) = &group[0].children {
            step.steps = wizard::level_steps(block, children);
        }
        let id = step.id.clone();
        let Some(siblings) = block.group_steps_mut(&level_path) else { continue };
        let at = insertion_index(siblings, level, &group);
        siblings.insert(at, step);
        touch(&id, &mut touched);
    }

    // Find numbers follow the new revision everywhere.
    for_each_step_mut(&mut block.steps, &mut |step| {
        for add in &mut step.adds {
            if let Some(c) = add.occurrences.first().and_then(|o| new_map.get(o.as_str())) {
                add.find_number = c.find_number.clone();
            }
        }
    });

    let retarget = Retarget {
        from: block.target.clone(),
        to,
        added: diff.added.clone(),
        removed: diff.removed.clone(),
        replaced: diff.replaced.clone(),
        touched_steps: touched,
        empty_steps: crate::edit::empty_steps(block),
    };
    block.retarget = Some(retarget.clone());
    Ok(retarget)
}

/// Approve the pending re-target: the new revision becomes the target and
/// steps the re-target left empty are deleted.
pub fn approve(block: &mut Instructions) -> Result<Target, String> {
    let retarget = block.retarget.take().ok_or("no re-target is waiting for approval")?;
    let empty: HashSet<&str> = retarget.empty_steps.iter().map(String::as_str).collect();
    retain_steps(&mut block.steps, &|s| !(empty.contains(s.id.as_str()) && s.adds.is_empty() && s.steps.is_empty()));
    block.target = retarget.to.clone();
    Ok(retarget.to)
}

fn touch(id: &str, touched: &mut Vec<String>) {
    if !touched.iter().any(|t| t == id) {
        touched.push(id.to_string());
    }
}

fn add_of(c: &Component) -> Add {
    Add {
        find_number: c.find_number.clone(),
        subassembly: c.children.is_some(),
        fastener: c.fastener,
        serial_tracked: c.serial_tracked,
        ..Add::named(vec![c.path.clone()], c.part_key(), &c.part)
    }
}

/// The path of the level that holds the component at `path`.
fn parent_level_path(root: &Level, path: &str) -> String {
    wizard::flatten(root).into_iter().find(|(_, c)| c.path == path).map(|(l, _)| l.to_string()).unwrap_or_default()
}

/// Where a new step for `group` goes among `steps` (the steps building its
/// level): after the last step that adds a part mated to it; otherwise a part
/// goes before the first fastener step and a fastener at the end.
fn insertion_index(steps: &[Step], level: &Level, group: &[&Component]) -> usize {
    let paths: HashSet<&str> = group.iter().map(|c| c.path.as_str()).collect();
    let neighbours: HashSet<&str> = level
        .mates
        .iter()
        .filter_map(|(a, b)| {
            if paths.contains(a.as_str()) {
                Some(b.as_str())
            } else if paths.contains(b.as_str()) {
                Some(a.as_str())
            } else {
                None
            }
        })
        .collect();
    let fastener = group[0].fastener;
    let last_neighbour = steps.iter().rposition(|s| s.adds.iter().any(|a| a.occurrences.iter().any(|o| neighbours.contains(o.as_str()))));
    match last_neighbour {
        Some(i) if !fastener => {
            // A part never goes after the fasteners.
            let first_fastener = steps.iter().position(|s| s.adds.iter().any(|a| a.fastener)).unwrap_or(steps.len());
            (i + 1).min(first_fastener)
        }
        Some(_) | None if fastener => steps.len(),
        _ => steps.iter().position(|s| s.adds.iter().any(|a| a.fastener)).unwrap_or(steps.len()),
    }
}

