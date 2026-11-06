//! The shared COMPONENT action set: the ONE list of what can be done to a
//! component, with each entry's availability and the reason when it is
//! refused, drawn by every surface a component is acted on from: the
//! selection context bar, the viewport's right-click menu and the BOM row
//! menu ([`component_menu`]); and the ONE dispatcher those surfaces call
//! ([`run_component_actions`]). Actions always route to the owning feature:
//! one truth, one undo step.
//!
//! Engine-mutating actions (Move, Unfix and Move, Move Copy, Fix/Unfix, Hide,
//! Delete) run HERE against [`EngineState`]; the document-level flows (Make
//! Work Part, Open part, Save part as, Replace, Info) return a
//! [`ComponentActionRequest`] for the SHELL, which owns the open documents,
//! the file dialog and the Info windows.

use brep_render::engine_state::EngineState;

use crate::column_tree::RowAction;

/// One per-component action.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ComponentAction {
    /// Make the component's part the Work part, in place in the assembly tab
    /// (shell-side, like the BOM row's double click).
    WorkPart,
    /// Open the part in a tab of its own: the live Work part session, so the
    /// assembly updates as the part is edited (shell-side). The ONE "Open
    /// part" flow every menu offers (owner 2026-11-05).
    OpenPart,
    /// A part with no file of its own (embedded): save it to a new part file
    /// (shell-side: the file dialog).
    SavePartAs,
    /// Roll the history to the component's ACOMP feature and open it there
    /// (shell-side: the history panel's focus).
    EditFeature,
    /// Toggle the Move gizmo on/off — arrows + rotation arcs together.
    /// Refused, with the reason, on a fixed component. Engine-side.
    Move,
    /// Unfix the fixed component and move it: unfixed only when the drag
    /// commits, in the move's own undo step (the same `isFixed`). Engine-side.
    UnfixMove,
    /// Add a SECOND instance of the same part (a new ACOMP on the source's
    /// pose, free even when the source is fixed) and arm the Move gizmo on it,
    /// so the user drags the copy into place. Engine-side.
    MoveCopy,
    /// Replace the part this occurrence shows with another (shell-side: the
    /// chooser lives in the shell). Mates survive by face name when the parts
    /// share lineage; the rest are held and listed.
    Replace,
    /// Fix ⇄ Unfix: the component's ONE fixed setting (`isFixed`, or
    /// `flexibleFixed` for a part inside a flexible sub-assembly), one undo
    /// step, the same command as `component_set_fixed`.
    ToggleFixed,
    /// Hide ⇄ Show the component's member solids. Engine-side.
    ToggleVisible,
    /// Open a pinned Info window per member solid (shell-side).
    Info,
    /// Delete the owning ACOMP feature (the library entry GC's kernel-side when
    /// its last instance goes).
    Delete,
}

impl ComponentAction {
    /// Every action, in menu order.
    pub const ALL: [ComponentAction; 12] = [
        ComponentAction::WorkPart,
        ComponentAction::OpenPart,
        ComponentAction::SavePartAs,
        ComponentAction::EditFeature,
        ComponentAction::Move,
        ComponentAction::UnfixMove,
        ComponentAction::MoveCopy,
        ComponentAction::Replace,
        ComponentAction::ToggleFixed,
        ComponentAction::ToggleVisible,
        ComponentAction::Info,
        ComponentAction::Delete,
    ];

    /// Stable id (widget keys `menuitem:<row>:<id>` / `component:<id>`).
    pub fn id(self) -> &'static str {
        match self {
            ComponentAction::WorkPart => "make-work-part",
            ComponentAction::OpenPart => "open-part",
            ComponentAction::SavePartAs => "save-part-as",
            ComponentAction::EditFeature => "edit-feature",
            ComponentAction::Move => "move",
            ComponentAction::UnfixMove => "unfix-move",
            ComponentAction::MoveCopy => "move-copy",
            ComponentAction::Replace => "replace",
            ComponentAction::ToggleFixed => "toggle-fixed",
            ComponentAction::ToggleVisible => "toggle-visible",
            ComponentAction::Info => "info",
            ComponentAction::Delete => "delete",
        }
    }

    /// The id in reverse (`None` for an unknown id).
    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|action| action.id() == id)
    }

    /// The label. `on` flips the two-state ones: Fix/Unfix (`on` = fixed) and
    /// Hide/Show (`on` = visible).
    pub fn label(self, on: bool) -> &'static str {
        match self {
            ComponentAction::WorkPart => "\u{25CE} Make Work Part",
            ComponentAction::OpenPart => "Open part",
            ComponentAction::SavePartAs => "Save part as\u{2026}",
            ComponentAction::EditFeature => "\u{270E} Edit feature",
            ComponentAction::Move => "\u{2725} Move",
            ComponentAction::UnfixMove => "\u{1F513} Unfix and Move",
            ComponentAction::MoveCopy => "\u{29C9} Move Copy",
            ComponentAction::Replace => "\u{21C4} Replace\u{2026}",
            ComponentAction::ToggleFixed if on => "\u{1F513} Unfix",
            ComponentAction::ToggleFixed => "\u{1F512} Fix",
            ComponentAction::ToggleVisible if on => "\u{1f441} Hide",
            ComponentAction::ToggleVisible => "\u{1f441} Show",
            ComponentAction::Info => "\u{1f575} Info",
            ComponentAction::Delete => "\u{2716} Delete",
        }
    }

    /// Hover tooltip.
    pub fn tooltip(self) -> &'static str {
        match self {
            ComponentAction::WorkPart => "Edit this part here, in the context of the assembly (double click)",
            ComponentAction::OpenPart => "Open this part in a tab of its own; the assembly updates as you edit",
            ComponentAction::SavePartAs => "This part has no file of its own: save it to a new part file you name",
            ComponentAction::EditFeature => "Roll to this component's feature and open it in the history",
            ComponentAction::Move => "Move/rotate gizmo on-off (arrows + arcs together)",
            ComponentAction::UnfixMove => {
                "Unfix this component and move it (unfixed only when you drag; one undo step)"
            }
            ComponentAction::MoveCopy => {
                "Add another instance of this part and drag it into place with the move gizmo"
            }
            ComponentAction::Replace => {
                "Show another part here (a family member, a variation, any part); mates keep their faces by name"
            }
            ComponentAction::ToggleFixed => "Fix this part in place, or free it, for the solver (its Fixed setting)",
            ComponentAction::ToggleVisible => "Hide or show this component",
            ComponentAction::Info => "Open a pinned Info window for this component",
            ComponentAction::Delete => "Delete this component instance",
        }
    }
}

/// A document-level flow the SHELL must run (it owns the open documents, the
/// file dialog and the Info windows); engine-mutating actions never produce
/// one.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum ComponentActionRequest {
    /// Save an embedded part to a new part file (the shell's part flow).
    SavePartAs { component_id: String },
    /// Open the Replace component chooser for this occurrence.
    Replace { component_id: String },
    /// Make `component_path` the Work part — in place inside the assembly
    /// tab, or (`in_place: false`) in a tab of its own.
    WorkPart { component_path: String, in_place: bool },
    /// Open one pinned Info window per entity name.
    Info { targets: Vec<String> },
    /// Focus the history on this feature (the menu already rolled to it).
    EditFeature { feature_id: String },
}

/// The Pattern Component instance menu's first, disabled line.
pub const PATTERN_INFO: &str = "pattern-info";

/// What a component menu stands for: the placements it acts on (one ACOMP
/// id, several for a packed BOM row, a nested occurrence path, or a Pattern
/// Component instance) and the facts its entries are decided on.
pub struct MenuSubject {
    pub ids: Vec<String>,
    /// The part has no file of its own (an embedded-only part): Save part as
    /// is offered.
    pub embedded: bool,
    /// Each placement's Fixed setting, or why it cannot be changed here (a
    /// part inside a rigid sub-assembly).
    pub fixed: Vec<Result<bool, String>>,
    /// Every member solid of every placement is visible.
    pub visible: bool,
}

impl MenuSubject {
    /// The subject for `ids`, read off the engine (a nested BOM line builds
    /// its own from the facts it already has, per frame, without a scan).
    pub fn of(state: &EngineState, ids: &[String]) -> Self {
        let instance = ids.first().and_then(|id| brep_render::brep_kernel::pattern_component_instance(id));
        let owner = |id: &String| match &instance {
            Some((_, seed, _)) => seed.clone(),
            None => id.clone(),
        };
        let embedded = ids.first().and_then(|id| state.component_info(&owner(id))).is_some_and(|info| {
            !state.part_source(&info.part_name).is_some_and(|(key, _)| !key.is_empty())
        });
        let fixed = ids
            .iter()
            .map(|id| {
                if instance.is_none() && id.contains(':') {
                    state.occurrence_fixed(id)
                } else {
                    state
                        .component_info(&owner(id))
                        .map(|info| info.fixed)
                        .ok_or_else(|| format!("'{id}' is not an assembly component"))
                }
            })
            .collect();
        Self { ids: ids.to_vec(), embedded, fixed, visible: all_visible(state, ids) }
    }
}

/// Every solid of the scene under `id` (`{id}:…`).
fn member_solids(state: &EngineState, id: &str) -> Vec<String> {
    let prefix = format!("{id}:");
    state
        .scene
        .solids()
        .iter()
        .filter(|solid| solid.name.starts_with(&prefix))
        .map(|solid| solid.name.clone())
        .collect()
}

/// Whether every member solid of every placement is visible.
fn all_visible(state: &EngineState, ids: &[String]) -> bool {
    ids.iter().all(|id| {
        let prefix = format!("{id}:");
        state.scene.solids().iter().filter(|solid| solid.name.starts_with(&prefix)).all(|solid| solid.visible)
    })
}

/// THE component menu: every [`ComponentAction`] in menu order, each enabled
/// or refused with its reason, for `subject`. The context bar, the viewport's
/// right-click menu and the BOM row menu all draw exactly this list.
pub fn component_menu(subject: &MenuSubject) -> Vec<RowAction> {
    let ids = &subject.ids;
    let Some(first) = ids.first() else { return Vec::new() };
    let instance = brep_render::brep_kernel::pattern_component_instance(first);
    let nested = instance.is_none() && first.contains(':');
    let rolled_up = ids.len() > 1;
    let n = ids.len();
    let fixed_states = &subject.fixed;
    let any_fixed = fixed_states.iter().any(|state| matches!(state, Ok(true)));
    let all_fixed = fixed_states.iter().all(|state| matches!(state, Ok(true)));
    let visible = subject.visible;
    let unpack = |verb: &str| format!("{n} placements on this row \u{2014} switch to Unpacked to {verb} one");
    let inside = "A part inside a sub-assembly: make the sub-assembly the Work part to do this";
    let copy = "A pattern copy: it follows its seed \u{2014} act on the seed";
    let mut menu = Vec::new();
    if let Some((pattern, seed, number)) = &instance {
        menu.push(
            RowAction::new(PATTERN_INFO, format!("Instance {number} of {pattern}: follows {seed}"))
                .disabled("A pattern copy: these act on its seed, and every copy follows"),
        );
    }
    for action in ComponentAction::ALL {
        let label = match action {
            ComponentAction::ToggleFixed => action.label(any_fixed),
            ComponentAction::ToggleVisible => action.label(visible),
            _ => action.label(false),
        };
        let entry = RowAction::new(action.id(), label).tooltip(action.tooltip());
        let entry = match action {
            ComponentAction::SavePartAs if !subject.embedded => continue,
            ComponentAction::UnfixMove if !any_fixed || nested || instance.is_some() => continue,
            ComponentAction::Move | ComponentAction::UnfixMove | ComponentAction::MoveCopy | ComponentAction::Delete
                if instance.is_some() =>
            {
                entry.disabled(copy)
            }
            ComponentAction::ToggleFixed if instance.is_some() => entry.disabled(copy),
            ComponentAction::Move
            | ComponentAction::UnfixMove
            | ComponentAction::MoveCopy
            | ComponentAction::Delete
            | ComponentAction::EditFeature
                if nested =>
            {
                entry.disabled(inside)
            }
            ComponentAction::Move if rolled_up => entry.disabled(unpack("move")),
            ComponentAction::UnfixMove if rolled_up => entry.disabled(unpack("move")),
            ComponentAction::MoveCopy if rolled_up => entry.disabled(unpack("copy")),
            ComponentAction::Delete if rolled_up => entry.disabled(unpack("delete")),
            ComponentAction::Move if all_fixed => {
                entry.disabled("This component is fixed \u{2014} Unfix it, or use Unfix and Move")
            }
            // A packed row's Fix/Unfix acts on EVERY placement it stands for,
            // as one undo step: Unfix frees the fixed ones (it reads Unfix when
            // ANY is), Fix fixes them all. A nested line standing for several
            // placements cannot (they live on different instances).
            ComponentAction::ToggleFixed if rolled_up && nested => entry.disabled(unpack("fix or unfix")),
            ComponentAction::ToggleFixed if rolled_up => entry.tooltip(if any_fixed {
                format!("Unfix the fixed placements among the {n} on this row (one undo step)")
            } else {
                format!("Fix all {n} placements on this row in place (one undo step)")
            }),
            ComponentAction::ToggleFixed => match fixed_states.first() {
                Some(Err(reason)) => entry.disabled(reason.clone()),
                _ => entry,
            },
            _ => entry,
        };
        menu.push(match action {
            // The destructive tail, fenced off from the rest.
            ComponentAction::Delete => entry.separator_above().destructive(),
            ComponentAction::EditFeature | ComponentAction::Move => entry.separator_above(),
            _ => entry,
        });
    }
    menu
}

/// Run `action` on one component (see [`run_component_actions`]).
pub fn run_component_action(
    state: &mut EngineState,
    action: ComponentAction,
    component_id: &str,
) -> Option<ComponentActionRequest> {
    run_component_actions(state, action, &[component_id.to_string()])
}

/// Run `action` on the placements a menu stands for (`ids`: one ACOMP id,
/// several for a packed BOM row, a nested path, or a pattern instance, which
/// acts on its seed). Engine-mutating actions apply immediately, one undo step;
/// a document-level flow comes back as the request the shell dispatches.
/// Refusals toast.
pub fn run_component_actions(
    state: &mut EngineState,
    action: ComponentAction,
    ids: &[String],
) -> Option<ComponentActionRequest> {
    let first = ids.first()?.clone();
    // On a pattern instance everything but Replace acts on its seed (Replace
    // keeps the instance id: the chooser opens on the seed and says why).
    let seed = brep_render::brep_kernel::pattern_component_instance(&first).map(|(_, seed, _)| seed);
    let target = match (&seed, action) {
        (Some(_), ComponentAction::Replace) | (None, _) => first.clone(),
        (Some(seed), _) => seed.clone(),
    };
    match action {
        ComponentAction::Move => {
            // Toggle the full gizmo (arrows + arcs together); a FIXED component
            // refuses with a toast inside the engine.
            state.component_move_toggle(&target);
            None
        }
        ComponentAction::UnfixMove => {
            state.component_move_unfix_toggle(&[target]);
            None
        }
        ComponentAction::MoveCopy => {
            // The copy is free even off a fixed source; the engine toasts an
            // unknown id itself.
            let _ = state.component_move_copy(&target);
            None
        }
        ComponentAction::ToggleFixed if ids.len() > 1 => {
            // A packed row: the same `isFixed` write across every placement,
            // one undo step. Unfix when any is fixed, else Fix them all.
            let any_fixed = ids.iter().any(|id| state.component_info(id).is_some_and(|info| info.fixed));
            if let Err(error) = state.set_components_fixed(ids, !any_fixed) {
                state.push_notice(error);
            }
            None
        }
        ComponentAction::ToggleFixed if target.contains(':') => {
            // A nested occurrence: fixed or freed on the instance that lends
            // it; a rigid level above it says why it cannot be.
            let toggled = state
                .occurrence_fixed(&target)
                .and_then(|fixed| state.set_component_fixed(&target, !fixed));
            if let Err(error) = toggled {
                state.push_notice(error);
            }
            None
        }
        ComponentAction::ToggleFixed => {
            let Some(info) = state.component_info(&target) else {
                state.push_notice(format!("'{target}' is not an assembly component"));
                return None;
            };
            // The ONE Fix/Unfix command (`component_set_fixed`): an explicit
            // `isFixed` boolean either way on the owning ACOMP, one undo step.
            if let Err(error) = state.set_component_fixed(&target, !info.fixed) {
                state.push_notice(error);
            }
            None
        }
        ComponentAction::ToggleVisible => {
            let ids: Vec<String> = if seed.is_some() { vec![first] } else { ids.to_vec() };
            let visible = !all_visible(state, &ids);
            let solids: Vec<String> = ids.iter().flat_map(|id| member_solids(state, id)).collect();
            state.set_visible_many(&solids, visible);
            None
        }
        ComponentAction::Info => {
            let ids: Vec<String> = if seed.is_some() { vec![first] } else { ids.to_vec() };
            let targets: Vec<String> = ids.iter().flat_map(|id| member_solids(state, id)).collect();
            (!targets.is_empty()).then_some(ComponentActionRequest::Info { targets })
        }
        ComponentAction::Delete => {
            // Deleting the feature is the ONE truth (every menu routes here);
            // the parts-library entry GC's at the kernel's next rebuild when its
            // last instance goes.
            let _ = state.delete_feature(&target);
            None
        }
        ComponentAction::SavePartAs => Some(ComponentActionRequest::SavePartAs { component_id: target }),
        ComponentAction::EditFeature => {
            let index = state.history.index_of(&target);
            match index {
                Some(index) => {
                    state.roll_to(index);
                    Some(ComponentActionRequest::EditFeature { feature_id: target })
                }
                None => {
                    state.push_notice(format!("'{target}' has no feature of its own"));
                    None
                }
            }
        }
        ComponentAction::Replace => Some(ComponentActionRequest::Replace { component_id: target }),
        ComponentAction::WorkPart | ComponentAction::OpenPart => Some(ComponentActionRequest::WorkPart {
            component_path: target,
            in_place: action == ComponentAction::WorkPart,
        }),
    }
}

/// The component's parts-library `sourceKey` (the ModelStore document name the
/// part was inserted from), read off the document's `partsLibrary` block —
/// `None` when the component / entry / key is absent (an imported or
/// embedded-only part). The Edit-Part flow offers to create a file from the
/// embedded document when a source is unavailable, and
/// keys its store lookup on this.
pub fn part_source_key(state: &EngineState, component_id: &str) -> Option<String> {
    let info = state.component_info(component_id)?;
    let document: serde_json::Value =
        serde_json::from_str(&state.history_request_json()).ok()?;
    document["partsLibrary"][&info.part_name]["sourceKey"]
        .as_str()
        .filter(|key| !key.is_empty())
        .map(str::to_string)
}

