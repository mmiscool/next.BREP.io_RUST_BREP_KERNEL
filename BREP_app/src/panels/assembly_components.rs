//! Assembly COMPONENT STATE — the engine's component projection as owned rows.
//!
//! Not a panel. This is the shared, panel-independent snapshot of "what
//! components does this document have, and what is true of each": label, fixed,
//! the outdated-vs-source flag, the constraint-status rollup, visibility,
//! selection, member solids, and the nested sub-assembly chain grouping.
//!
//! It was the Assembly Structure panel's private snapshot until that panel was
//! folded into the BOM (which now draws every one of these adornments as a
//! column or an Item-cell glyph). The panel is gone; the projection outlived it
//! because it is engine truth, not a view: the BOM reads it, and the headed
//! verifiers read `__brepAssemblyTree` published from it.
//!
//! Strictly a VIEW projected from [`EngineState::assembly_components`] — never
//! an owning structure. The single source of truth stays history + parts
//! library, so undo/redo and provenance keep working unchanged.

use crate::panels::update_components::UpdateComponents;
use brep_render::assembly_status;
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// The grounded-component glyph (matches the kernel's Fixed-constraint glyph).
pub(crate) const FIXED_GLYPH: &str = "\u{23DA}"; // ⏚ earth ground

/// The outdated-vs-source badge glyph (`↻` — "refresh me").
pub(crate) const OUTDATED_GLYPH: &str = "\u{21BB}";

/// The outdated badge's amber (the status map's warning amber `#ff9f0a`).
pub(crate) const OUTDATED_AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0x9f, 0x0a);

/// One component row's per-frame snapshot (decoupled from `state` so the draw
/// loop can issue deferred `&mut state` mutations afterwards — the panel
/// pattern shared with the Scene tree).
pub(crate) struct ComponentRow {
    /// Owning ACOMP feature id (= namespace prefix).
    pub id: String,
    /// `part_name (ACOMP3)`.
    pub label: String,
    /// The library part this instance is of — the BOM groups by it.
    pub part_name: String,
    pub fixed: bool,
    /// The instance's library entry no longer matches its store source (the
    /// update-components checker's per-part flag — every instance lights).
    pub outdated: bool,
    /// Worst constraint status referencing this component (None = the
    /// component participates in no constraint — no rollup dot).
    pub rollup_status: Option<String>,
    /// Every member solid currently visible?
    pub visible: bool,
    /// Any member solid in the current selection (viewport → tree sync)?
    pub selected: bool,
    /// Member scene names (namespaced).
    pub solids: Vec<String>,
    /// Read-only nested sub-assembly grouping parsed from the member names.
    pub children: Vec<ChainNode>,
}

/// One node of a rigid sub-assembly's read-only child grouping, parsed from the
/// members' chained namespace prefixes (`ACOMP3:ACOMP1:Part` → child `ACOMP1`
/// containing leaf `Part`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ChainNode {
    pub label: String,
    pub children: Vec<ChainNode>,
}

/// Whether a name segment reads as a component id (`ACOMP<digits>`) — the
/// syntactic nested-namespace discriminator (inner ids are not scene
/// components, so membership can only be judged by shape).
pub(crate) fn is_acomp_segment(segment: &str) -> bool {
    segment
        .strip_prefix("ACOMP")
        .map(|digits| !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit()))
        .unwrap_or(false)
}

/// The `collapsed`-set key for a nested component node: its chained path under
/// the owning component (`ACOMP1:ACOMP5`). Top-level rows key on the bare
/// `row.id` (never contains `:`); nested nodes append `:label`. The ONE place
/// this key is formed, so the render path and the collapse-all enumerator
/// ([`collapsible_keys`]) can never drift.
fn chain_key(parent_key: &str, label: &str) -> String {
    format!("{parent_key}:{label}")
}

/// Parse the read-only child grouping of one component's members: each member
/// name arrives with the OWNING prefix already stripped; a leading
/// `ACOMP<digits>:` chain groups into nested nodes, the remainder is a leaf.
/// Deterministic: groups in first-appearance order, leaves in member order.
pub(crate) fn chain_groups(member_locals: &[&str]) -> Vec<ChainNode> {
    let mut order: Vec<String> = Vec::new();
    let mut grouped: HashMap<String, Vec<&str>> = HashMap::new();
    let mut leaves: Vec<ChainNode> = Vec::new();
    for local in member_locals {
        match local.split_once(':') {
            Some((head, rest)) if is_acomp_segment(head) => {
                if !grouped.contains_key(head) {
                    order.push(head.to_string());
                }
                grouped.entry(head.to_string()).or_default().push(rest);
            }
            _ => leaves.push(ChainNode {
                label: (*local).to_string(),
                children: Vec::new(),
            }),
        }
    }
    let mut out: Vec<ChainNode> = order
        .into_iter()
        .map(|head| {
            let members = grouped.remove(&head).unwrap_or_default();
            ChainNode {
                children: chain_groups(&members),
                label: head,
            }
        })
        .collect();
    out.append(&mut leaves);
    out
}

/// The component id an assembly-constraint element ref belongs to: the vertex
/// `@`-suffix is stripped first, then the OUTERMOST namespace prefix (or the
/// bare id itself) is the owner. `None` for non-component refs.
fn owning_component_of_ref(element: &str) -> Option<&str> {
    let name = element.split('@').next().unwrap_or(element);
    let head = name.split(':').next().unwrap_or(name);
    is_acomp_segment(head).then_some(head)
}


/// Snapshot the engine's component projection into owned rows: label, fixed,
/// the outdated flag (from the update-components checker, per part name),
/// visibility (every member visible), selection (any member selected), the
/// constraint-status rollup, and the nested chain grouping.
pub(crate) fn snapshot(state: &mut EngineState, updates: &UpdateComponents) -> Vec<ComponentRow> {
    // Worst-status rollup per component, from the constraint state's element
    // refs (each ref's outermost prefix names its component).
    let mut rollup: HashMap<String, String> = HashMap::new();
    let constraint_state = state.assembly_state_value();
    if let Some(constraints) = constraint_state
        .get("constraints")
        .and_then(Value::as_array)
    {
        for entry in constraints {
            let status = entry
                .get("persistentData")
                .and_then(|data| data.get("status"))
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string();
            let elements = entry
                .get("inputParams")
                .and_then(|params| params.get("elements"))
                .and_then(Value::as_array)
                .cloned()
                .unwrap_or_default();
            for element in elements.iter().filter_map(Value::as_str) {
                let Some(component) = owning_component_of_ref(element) else {
                    continue;
                };
                let worse = rollup
                    .get(component)
                    .map(|current| {
                        assembly_status::status_severity(&status)
                            > assembly_status::status_severity(current)
                    })
                    .unwrap_or(true);
                if worse {
                    rollup.insert(component.to_string(), status.clone());
                }
            }
        }
    }

    let selected_solids = state.emphasis.selected_solids.clone();
    let solid_visible: HashMap<String, bool> = state
        .scene
        .solids()
        .iter()
        .map(|solid| (solid.name.clone(), solid.visible))
        .collect();

    state
        .assembly_components()
        .iter()
        .map(|record| {
            let prefix = format!("{}:", record.id);
            let locals: Vec<&str> = record
                .solids
                .iter()
                .map(|name| name.strip_prefix(&prefix).unwrap_or(name))
                .collect();
            ComponentRow {
                label: format!("{} ({})", record.part_name, record.id),
                part_name: record.part_name.clone(),
                fixed: record.fixed,
                outdated: updates.is_outdated(&record.part_name),
                rollup_status: rollup.get(&record.id).cloned(),
                visible: record
                    .solids
                    .iter()
                    .all(|name| solid_visible.get(name).copied().unwrap_or(true)),
                selected: record
                    .solids
                    .iter()
                    .any(|name| selected_solids.contains(name)),
                solids: record.solids.clone(),
                children: chain_groups(&locals),
                id: record.id.clone(),
            }
        })
        .collect()
}

/// Publish `__brepAssemblyTree` — the headed verifiers' component oracle.
///
/// Published from the projection rather than from any panel, so it stays true
/// whichever view is on screen. Three verifiers read it, one of them
/// (`verify_bom_menu`) as the ENGINE-SIDE proof that a menu action landed —
/// checking a panel's own rendering would be checking the panel against itself.
#[allow(unused_variables)]
pub(crate) fn publish_tree(rows: &[ComponentRow]) {
    if crate::automation::registry::enabled() {
        let listing: Vec<Value> = rows
            .iter()
            .map(|row| {
                serde_json::json!({
                    "id": row.id,
                    "label": row.label,
                    "fixed": row.fixed,
                    "outdated": row.outdated,
                    "visible": row.visible,
                    "selected": row.selected,
                    "status": row.rollup_status,
                    "solids": row.solids,
                })
            })
            .collect();
        crate::automation::registry::publish("__brepAssemblyTree", "assembly structure tree rows", &Value::Array(listing).to_string());
    }
}
