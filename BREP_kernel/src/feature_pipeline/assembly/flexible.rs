//! Flexible sub-assemblies.
//!
//! A sub-assembly instance is rigid by default: its own mates solve inside its
//! isolated run and the parent places it as one body. An instance marked
//! `flexible` instead lends its occurrences to the PARENT'S solve:
//!
//! * [`expand`] registers each free occurrence of the instance as a solver body
//!   (`SceneMap::occurrences`, id `ACOMP1:ACOMP2`, posed at the instance frame
//!   composed with the per-instance seed, the child's solved pose, or its
//!   authored pose) and adds the child's own constraints, namespaced, to the
//!   state being solved. Occurrences the child grounds stay with the
//!   instance's frame. An occurrence the child itself places flexible lends
//!   its own occurrences and constraints the same way, at any depth, under
//!   the full path (`ACOMP1:ACOMP1:ACOMP2`).
//! * the lifecycle then solves everything as one system, so a parent mate can
//!   move an occurrence along whatever its child mates leave free;
//! * [`collect`] takes the child's constraints back out of the state (the
//!   parent's document never stores them) and reports them as the instance's
//!   nested rows.
//!
//! The solved occurrence poses persist on the placing feature
//! (`inputParams.flexiblePoses`, part-local), never in the child document.

use super::{AssemblyState, ConstraintEntry};
use crate::feature_pipeline::component::{
    compose, namespaced, update_component_transform, ComponentRecord, FlexibleSpec, NestedConstraint,
};
use crate::feature_pipeline::SceneMap;

/// Register the occurrences of every flexible instance (once per scene) and
/// add their child constraints to `state` (once per state).
pub(super) fn expand(state: &mut AssemblyState, scene: &mut SceneMap) {
    let flexible: Vec<ComponentRecord> = scene
        .components
        .values()
        .filter(|record| record.flexible.is_some())
        .cloned()
        .collect();
    for record in &flexible {
        let Some(spec) = &record.flexible else { continue };
        lend(state, scene, record, spec, "");
    }
}

/// One level of a flexible instance: `spec` describes the document placed at
/// `prefix` inside `record` (`""` for the instance's own child, `ACOMP1:` for
/// an occurrence that child places flexible, and so on down). Its free
/// occurrences become solver bodies and its constraints join `state`, both
/// under the full path; a flexible occurrence lends its own level in turn.
fn lend(state: &mut AssemblyState, scene: &mut SceneMap, record: &ComponentRecord, spec: &FlexibleSpec, prefix: &str) {
    let level = |path: &str| path.strip_prefix(prefix).filter(|local| !local.contains(':')).map(str::to_string);
    let mut locals: Vec<String> = record.nested_poses.keys().filter_map(|path| level(path)).collect();
    if prefix.is_empty() {
        locals.extend(spec.authored.keys().cloned());
    }
    locals.sort();
    locals.dedup();
    for local in locals {
        let path = format!("{prefix}{local}");
        // The instance's own Fix/Float wins over the child's grounding.
        match record.flexible.as_ref().and_then(|top| top.fixed.get(&path)) {
            Some(&fixed) => register(scene, record, spec, &path, fixed),
            None if !spec.grounded.contains(&local) => register(scene, record, spec, &path, false),
            None => {}
        }
        if let Some(inner) = spec.nested.get(&local) {
            lend(state, scene, record, inner, &format!("{path}:"));
        }
    }
    let owner = match prefix.strip_suffix(':') {
        Some(path) => namespaced(&record.id, path),
        None => record.id.clone(),
    };
    for raw in &spec.constraints {
        let Ok(mut entry) = serde_json::from_value::<ConstraintEntry>(raw.clone()) else {
            continue;
        };
        // A Fixed constraint grounds an occurrence: that is `grounded`.
        if entry.constraint_type == "fixed" {
            continue;
        }
        let id = namespaced(&owner, entry.id());
        if state.constraints.iter().any(|existing| existing.id() == id) {
            continue;
        }
        let elements: Vec<serde_json::Value> = entry
            .elements()
            .iter()
            .map(|element| serde_json::Value::String(namespaced(&owner, element)))
            .collect();
        if let Some(params) = entry.input_params.as_object_mut() {
            params.insert("id".into(), serde_json::Value::String(id));
            params.insert("elements".into(), serde_json::Value::Array(elements));
        }
        state.constraints.push(entry);
    }
}

/// Register the occurrence at `path` (instance-local) of `record` as a solver
/// body (grounded when `fixed`), posed at the instance frame composed with its
/// seed, its solved pose in the child, or (one level down) where the child
/// authors it.
fn register(scene: &mut SceneMap, record: &ComponentRecord, spec: &FlexibleSpec, path: &str, fixed: bool) {
    let id = namespaced(&record.id, path);
    if scene.occurrences.contains_key(&id) {
        return;
    }
    let prefix = format!("{id}:");
    let solids: Vec<String> = record.solids.iter().filter(|name| name.starts_with(&prefix)).cloned().collect();
    let Some(local) = record.nested_poses.get(path).or_else(|| spec.authored.get(path)) else {
        return;
    };
    if solids.is_empty() {
        return;
    }
    let Ok(transform) = compose(&record.transform, local) else { return };
    scene.occurrences.insert(
        id.clone(),
        ComponentRecord {
            id: id.clone(),
            part_name: path.to_string(),
            transform,
            fixed,
            source: serde_json::Value::Null,
            solids,
            ports: Vec::new(),
            nested_constraints: Vec::new(),
            nested_poses: Default::default(),
            pattern: None,
            flexible: None,
        },
    );
    // The instance's persisted pose for this occurrence, if any.
    let seed = record.flexible.as_ref().and_then(|top| top.seeds.get(path));
    if let Some(seed) = seed {
        if let Ok(seeded) = compose(&record.transform, seed) {
            let _ = update_component_transform(scene, &id, seeded);
        }
    }
}

/// Take the flexible instances' child constraints back out of `state` and
/// report them on their instances as nested rows (replacing the rows the
/// child's own isolated solve produced).
pub(super) fn collect(state: &mut AssemblyState, scene: &mut SceneMap) {
    let ids: Vec<String> = scene
        .components
        .values()
        .filter(|record| record.flexible.is_some())
        .map(|record| record.id.clone())
        .collect();
    for id in ids {
        let prefix = format!("{id}:");
        let (taken, kept): (Vec<ConstraintEntry>, Vec<ConstraintEntry>) =
            std::mem::take(&mut state.constraints)
                .into_iter()
                .partition(|entry| entry.id().starts_with(&prefix));
        state.constraints = kept;
        let Some(record) = scene.components.get_mut(&id) else { continue };
        for entry in &taken {
            let row = NestedConstraint::from_entry(entry);
            match record.nested_constraints.iter_mut().find(|existing| existing.id == row.id) {
                Some(existing) => *existing = row,
                None => record.nested_constraints.push(row),
            }
        }
    }
}

/// The local (instance-frame) pose of an occurrence at world pose `transform`,
/// or `None` when `id` is not a flexible occurrence.
pub(super) fn occurrence_local(
    scene: &SceneMap,
    id: &str,
    transform: &crate::AffineTransform,
) -> Option<crate::AffineTransform> {
    scene.occurrences.get(id)?;
    let (owner, _) = id.split_once(':')?;
    let parent = scene.components.get(owner)?;
    compose(&parent.transform.rigid_inverse().ok()?, transform).ok()
}
