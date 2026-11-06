//! THE delete-selection command: what the Delete / Backspace key, the context
//! bar's Delete entries and MCP `selection_delete` all run.
//!
//! One question — what does "delete the selection" mean right now? — answered
//! once (`target`), then one dispatch (`apply`) to the delete each kind
//! already has: the same call the row ✕ / menu Delete makes, so the same (no)
//! confirmation and the same single undo step. The modes are exclusive, in the
//! order a user is inside them: an open sketch owns the key, then a 3D sketch,
//! an eCAD editor, an open drawing sheet, the PMI workbench; otherwise the 3D
//! selection is an assembly constraint (label-picked), one component (its
//! member solids picked in the view or its BOM row), or one feature (its
//! history-tree row). A sheet or PMI mode with nothing selected deletes
//! nothing — it never falls through to a feature the user cannot see.

use crate::document::Documents;
use brep_render::engine_state::EngineState;

/// What the delete command acts on.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeleteTarget {
    /// The open sketch's selected geometry / points / constraints.
    Sketch,
    /// The open 3D sketch's selection.
    Sketch3d,
    /// The eCAD editor on the central tile (its own selection).
    Ecad(crate::workbench::ecad::Target),
    /// A held object on the open drawing sheet (view, dimension, note…).
    SheetObject(String),
    /// The selected PMI annotation.
    PmiAnnotation(String),
    /// The label-selected assembly constraint.
    Constraint(String),
    /// One assembly component (its ACOMP feature).
    Component(String),
    /// One history feature.
    Feature(String),
}

impl DeleteTarget {
    /// The kind word MCP reports.
    pub fn kind(&self) -> &'static str {
        match self {
            DeleteTarget::Sketch => "sketch",
            DeleteTarget::Sketch3d => "sketch3d",
            DeleteTarget::Ecad(_) => "ecad",
            DeleteTarget::SheetObject(_) => "sheetObject",
            DeleteTarget::PmiAnnotation(_) => "pmiAnnotation",
            DeleteTarget::Constraint(_) => "constraint",
            DeleteTarget::Component(_) => "component",
            DeleteTarget::Feature(_) => "feature",
        }
    }

    /// The id deleted, for the kinds that name one.
    pub fn id(&self) -> Option<&str> {
        match self {
            DeleteTarget::SheetObject(id)
            | DeleteTarget::PmiAnnotation(id)
            | DeleteTarget::Constraint(id)
            | DeleteTarget::Component(id)
            | DeleteTarget::Feature(id) => Some(id),
            _ => None,
        }
    }
}

/// What "delete the selection" means for `docs` now; `None` when nothing is
/// deletable.
pub fn target(docs: &Documents) -> Option<DeleteTarget> {
    let engine = docs.engine();
    if engine.ref_select_active() {
        // A reference pick owns the selection: Delete never edits the model
        // under a picking form.
        return None;
    }
    if engine.sketch_mode() {
        return Some(DeleteTarget::Sketch);
    }
    if engine.sketch3d_mode() {
        return Some(DeleteTarget::Sketch3d);
    }
    if let Some(target) = crate::workbench::ecad::Target::of_workbench(&engine.settings.workbench) {
        return Some(DeleteTarget::Ecad(target));
    }
    if engine.sheet_open().is_some() {
        return engine.sheet_selected_object().map(|id| DeleteTarget::SheetObject(id.to_string()));
    }
    if engine.pmi_workbench_entered() {
        return engine.pmi_selected_annotation().map(|id| DeleteTarget::PmiAnnotation(id.to_string()));
    }
    model_target(engine)
}

/// The 3D-model part of [`target`]: constraint, then component, then feature.
pub(crate) fn model_target(engine: &EngineState) -> Option<DeleteTarget> {
    let constraints_shown = crate::workbench::panel_visible(
        &engine.settings.workbench,
        crate::workbench::assembly::CONSTRAINTS_PANEL_ID,
        &crate::workbench::ButtonState::of(engine),
    );
    if let Some(id) = engine.selected_constraint().filter(|_| constraints_shown) {
        return Some(DeleteTarget::Constraint(id));
    }
    if let Some(id) = crate::panels::context_bar::component_action_target_of(engine) {
        return Some(DeleteTarget::Component(id));
    }
    engine.selected_feature().map(|id| DeleteTarget::Feature(id.to_string()))
}

/// Delete `target` through the delete its kind already has. `Err` carries the
/// refusal the row action would have shown.
pub fn apply(docs: &mut Documents, target: &DeleteTarget) -> Result<(), String> {
    match target {
        DeleteTarget::Ecad(target) => {
            docs.active_mut().ecad.selection_key(*target, true);
            Ok(())
        }
        other => apply_engine(docs.engine_mut(), other),
    }
}

/// [`apply`] for every kind the engine alone holds (all but eCAD, which lives
/// on the document) — what an engine-only surface such as the context bar
/// calls, so its Delete entries are this command too.
pub(crate) fn apply_engine(engine: &mut EngineState, target: &DeleteTarget) -> Result<(), String> {
    match target {
        DeleteTarget::Sketch => {
            engine.sketch_delete_selection();
            Ok(())
        }
        DeleteTarget::Sketch3d => {
            engine.sketch3d_delete_selection();
            Ok(())
        }
        DeleteTarget::Ecad(_) => Err("the eCAD editor's selection is deleted in its editor".into()),
        DeleteTarget::SheetObject(id) => crate::panels::sheets::delete_held_object(engine, id),
        DeleteTarget::PmiAnnotation(id) => crate::panels::pmi::delete_annotation(engine, id),
        DeleteTarget::Constraint(id) => {
            let result = engine.assembly_remove_constraint(id);
            engine.constraint_deselect();
            result
        }
        DeleteTarget::Component(id) => {
            if id.contains(':') {
                return Err(format!(
                    "'{id}' is a part inside a sub-assembly: open that sub-assembly to delete it"
                ));
            }
            crate::panels::component_actions::run_component_action(
                engine,
                crate::panels::component_actions::ComponentAction::Delete,
                id,
            );
            Ok(())
        }
        DeleteTarget::Feature(id) => {
            engine.delete_feature(id);
            Ok(())
        }
    }
}

/// [`target`] then [`apply`]; `Ok(None)` when nothing was deletable.
pub fn delete_selection(docs: &mut Documents) -> Result<Option<DeleteTarget>, String> {
    let Some(target) = target(docs) else {
        return Ok(None);
    };
    apply(docs, &target)?;
    Ok(Some(target))
}

