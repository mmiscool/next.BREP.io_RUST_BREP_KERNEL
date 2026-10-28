//! The shared COMPONENT action set — the ONE entry point for per-component
//! interactions, consumed by the selection context bar (this lane) and,
//! through the same enum, by the assembly structure tree's per-node action
//! hooks (lane F seam: the tree calls [`run_component_action`] with the
//! node's owning ACOMP feature id — actions always ROUTE TO the owning
//! feature, one truth, one undo lane).
//!
//! Engine-mutating actions (Move / Fix-Unfix / Delete) run HERE against
//! [`EngineState`]; the document-level flow (Edit Part) returns a
//! [`ComponentActionRequest`] for the SHELL, which owns the open documents and
//! the file dialog.

use brep_render::engine_state::EngineState;

/// One per-component action.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ComponentAction {
    /// Toggle the Move gizmo on/off — arrows + rotation arcs together (fixed refuses
    /// with a toast). Engine-side.
    Move,
    /// Add a SECOND instance of the same part (a new ACOMP on the source's
    /// pose, free even when the source is fixed) and arm the Move gizmo on it,
    /// so the user drags the copy into place. Engine-side.
    MoveCopy,
    /// Open the part's SOURCE document in its own document tab (shell-side).
    /// Editing a component IS opening its part: there is no separate
    /// edit-in-place session any more — the part is a document like any other,
    /// and the assembly picks the change up through the outdated badge /
    /// `panels::update_components` once the part is saved.
    OpenPart,
    /// Make the component's part the Work part, in place in the assembly tab
    /// (shell-side, like the BOM row's double click).
    WorkPart,
    /// Fix ⇄ Unfix (writes `isFixed` on the owning ACOMP; re-runs + re-solves).
    ToggleFixed,
    /// Replace the part this occurrence shows with another (shell-side: the
    /// chooser lives in the shell). Mates survive by face name when the parts
    /// share lineage; the rest are held and listed.
    Replace,
    /// Delete the owning ACOMP feature (the library entry GC's kernel-side when
    /// its last instance goes).
    Delete,
}

impl ComponentAction {
    /// Every action, in bar order.
    pub const ALL: [ComponentAction; 7] = [
        ComponentAction::Move,
        ComponentAction::MoveCopy,
        ComponentAction::OpenPart,
        ComponentAction::WorkPart,
        ComponentAction::Replace,
        ComponentAction::ToggleFixed,
        ComponentAction::Delete,
    ];

    /// Stable id (widget keys / verifier state).
    pub fn id(self) -> &'static str {
        match self {
            ComponentAction::Move => "move",
            ComponentAction::MoveCopy => "move-copy",
            ComponentAction::OpenPart => "open-part",
            ComponentAction::WorkPart => "work-part",
            ComponentAction::Replace => "replace",
            ComponentAction::ToggleFixed => "toggle-fixed",
            ComponentAction::Delete => "delete",
        }
    }

    /// The id in reverse (`None` for an unknown id).
    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|action| action.id() == id)
    }

    /// The button label. `fixed` flips the Fix/Unfix wording.
    pub fn label(self, fixed: bool) -> &'static str {
        match self {
            ComponentAction::Move => "\u{2725} Move",
            ComponentAction::MoveCopy => "\u{29C9} Move Copy",
            ComponentAction::OpenPart => "\u{270E} Edit Part",
            ComponentAction::WorkPart => "\u{25CE} Make Work Part",
            ComponentAction::Replace => "\u{21C4} Replace\u{2026}",
            ComponentAction::ToggleFixed => {
                if fixed {
                    "\u{1F513} Unfix"
                } else {
                    "\u{1F512} Fix"
                }
            }
            ComponentAction::Delete => "\u{2716} Delete",
        }
    }

    /// Hover tooltip.
    pub fn tooltip(self) -> &'static str {
        match self {
            ComponentAction::Move => "Move/rotate gizmo on-off (arrows + arcs together)",
            ComponentAction::MoveCopy => {
                "Add another instance of this part and drag it into place with the move gizmo"
            }
            ComponentAction::OpenPart => "Open the part's source document in its own tab",
            ComponentAction::WorkPart => "Edit this part here, in the context of the assembly",
            ComponentAction::Replace => {
                "Show another part here (a family member, a variation, any part); mates keep their faces by name"
            }
            ComponentAction::ToggleFixed => "Ground / free this instance for the solver",
            ComponentAction::Delete => "Delete this component instance",
        }
    }
}

/// A document-level flow the SHELL must run (it owns the open documents + the
/// file dialog); engine-mutating actions never produce one.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum ComponentActionRequest {
    OpenPart { component_id: String },
    /// Open the Replace component chooser for this occurrence.
    Replace { component_id: String },
    /// Make `component_path` the Work part — in place inside the assembly
    /// tab, or (`in_place: false`) in a tab of its own.
    WorkPart { component_path: String, in_place: bool },
}

/// Run `action` on the component owned by ACOMP feature `component_id`.
/// Engine-mutating actions apply immediately; Edit Part returns the request the
/// shell dispatches. Unknown component ids toast.
pub fn run_component_action(
    state: &mut EngineState,
    action: ComponentAction,
    component_id: &str,
) -> Option<ComponentActionRequest> {
    match action {
        ComponentAction::Move => {
            // Toggle the full gizmo (arrows + arcs together); a FIXED component
            // refuses with a toast inside the engine.
            state.component_move_toggle(component_id);
            None
        }
        ComponentAction::MoveCopy => {
            // The copy is free even off a fixed source; the engine toasts an
            // unknown id itself.
            let _ = state.component_move_copy(component_id);
            None
        }
        ComponentAction::ToggleFixed if component_id.contains(':') => {
            // A nested occurrence: fixed or freed on the instance that lends
            // it; a rigid level above it says why it cannot be.
            let toggled = state
                .occurrence_fixed(component_id)
                .and_then(|fixed| state.set_component_fixed(component_id, !fixed));
            if let Err(error) = toggled {
                state.push_notice(error);
            }
            None
        }
        ComponentAction::ToggleFixed => {
            let Some(info) = state.component_info(component_id) else {
                state.push_notice(format!("'{component_id}' is not an assembly component"));
                return None;
            };
            let mut params = serde_json::from_str::<serde_json::Value>(
                &state.feature_params_json(feature_index(state, component_id)?),
            )
            .unwrap_or_else(|_| serde_json::json!({}));
            if let Some(object) = params.as_object_mut() {
                // An EXPLICIT boolean either way — the kernel honors explicit
                // `false` (un-fixing the sole component stays possible; the
                // absent-auto-grounds rule keys on absence only).
                object.insert("isFixed".into(), serde_json::Value::Bool(!info.fixed));
            }
            let _ = state.update_feature_params(component_id, &params.to_string());
            None
        }
        ComponentAction::Delete => {
            // Deleting the feature is the ONE truth (tree/bar both route here);
            // the parts-library entry GC's at the kernel's next rebuild when its
            // last instance goes.
            let _ = state.delete_feature(component_id);
            None
        }
        ComponentAction::OpenPart => Some(ComponentActionRequest::OpenPart {
            component_id: component_id.to_string(),
        }),
        ComponentAction::Replace => Some(ComponentActionRequest::Replace {
            component_id: component_id.to_string(),
        }),
        ComponentAction::WorkPart => Some(ComponentActionRequest::WorkPart {
            component_path: component_id.to_string(),
            in_place: true,
        }),
    }
}

/// The feature index carrying id `id` (the engine exposes index→id, so scan).
fn feature_index(state: &EngineState, id: &str) -> Option<usize> {
    (0..state.history_len()).find(|&i| state.feature_id_at(i).as_deref() == Some(id))
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

