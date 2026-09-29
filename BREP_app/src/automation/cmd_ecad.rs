//! The eCAD workbenches: eCAD's own actions by id, and the editor's state.
//!
//! `workbench_button` presses what the Diagram, PCB, Symbol and Pads rows
//! carry. These reach the rest of eCAD's action tables — zoom, the selection's
//! commands (delete, duplicate, cancel), the actions that only put a tool down
//! — by eCAD's own id, gated by eCAD's own `offered` and `enabled`, as a key or
//! a click on eCAD's toolbar would be. The one group refused is History: eCAD's
//! own undo history would be a second one beside the document's, which is
//! exactly what the host keeps shut (`handle_shortcuts` takes the history keys
//! before any editor sees them), so the command names `undo` instead.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::workbench::ecad::Target;
use brep_ecad_egui::{Action, ActionGroup};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EcadActionArgs {
    /// eCAD's own action id, e.g. `sheet.delete`, `board.cancel`,
    /// `symbol.tool.pin`. A Diagram or PCB row button's id
    /// (`pcb.sheet.tool.wire`) is accepted too, with its workbench prefix.
    pub id: String,
}

/// Run `id` from `table` on `editor`, or say why not — naming the id, and
/// listing what the editor offers now.
fn run<T: 'static>(
    workbench: &str,
    editor: &mut T,
    table: Vec<&'static Action<T>>,
    id: &str,
    run_action: fn(&mut T, &str) -> bool,
) -> Result<Outcome, String> {
    let offered = || {
        table
            .iter()
            .filter(|a| a.group != ActionGroup::History && a.offered(editor))
            .map(|a| a.id)
            .collect::<Vec<_>>()
            .join(", ")
    };
    let Some(action) = table.iter().find(|a| a.id == id) else {
        return Err(format!("`{id}` is not an action of the {workbench} editor; offered now: {}", offered()));
    };
    if action.group == ActionGroup::History {
        return Err(format!(
            "`{id}` is eCAD's own undo history, which BREP does not use: the document keeps ONE undo history — use `undo` / `redo`"
        ));
    }
    if !action.offered(editor) {
        return Err(format!("`{id}` is not offered in the {workbench} editor now; offered: {}", offered()));
    }
    if !action.enabled(editor) {
        return Err(format!("`{id}` is offered but not enabled now (\"{}\" needs something it does not have yet, such as a selection)", action.label));
    }
    if !run_action(editor, id) {
        return Err(format!("`{id}` did not run"));
    }
    Ok(Outcome::Done(json!({ "id": id, "workbench": workbench })))
}

fn ecad_action(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: EcadActionArgs = parse_args(args)?;
    let doc = ctx.app.docs.active_mut();
    let workbench = doc.engine.settings.workbench.clone();
    let Some(target) = Target::of_workbench(&workbench) else {
        return Err(format!(
            "`{}`: no eCAD workbench is active (active: `{}`) — choose diagram, pcb, symbol or pads",
            a.id,
            crate::workbench::resolve(&workbench).id
        ));
    };
    // A row button's id carries its workbench in front; eCAD's does not.
    let id = a.id.strip_prefix(target.workbench_id()).and_then(|rest| rest.strip_prefix('.'));
    let id = match target {
        Target::Diagram | Target::Pcb => id.unwrap_or(&a.id),
        // The symbol and pads editors' ids already lead with their workbench.
        Target::Symbol | Target::Pads => &a.id,
    };
    let name = target.workbench_id();
    match target {
        Target::Diagram => run(name, &mut doc.ecad.diagram, brep_ecad_egui::actions().collect(), id, brep_ecad_egui::Editor::run_action),
        Target::Pcb => run(name, &mut doc.ecad.pcb, brep_ecad_egui::actions().collect(), id, brep_ecad_egui::Editor::run_action),
        Target::Symbol => run(
            name,
            &mut doc.ecad.symbol,
            brep_ecad_egui::symbol_actions().iter().collect(),
            id,
            brep_ecad_egui::SymbolEditor::run_action,
        ),
        Target::Pads => run(
            name,
            &mut doc.ecad.pads,
            brep_ecad_egui::pad_actions().iter().collect(),
            id,
            brep_ecad_egui::FootprintEditor::run_action,
        ),
    }
}

fn ecad_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let doc = ctx.app.docs.active();
    let Some(target) = Target::of_workbench(&doc.engine.settings.workbench) else {
        return Ok(Outcome::Done(json!({ "target": null })));
    };
    let block = crate::viewport::ecad::block_of(target);
    let sync = doc
        .ecad_blocks
        .iter()
        .find(|sync| sync.block() == block)
        .expect("a document keeps one BlockSync per block");
    let mut state = crate::viewport::ecad::state_json(target, &doc.ecad, sync, &doc.engine.history);
    state["status"] = crate::viewport::ecad::status_json(ctx.app.viewport.ecad_status());
    state["outdated"] = crate::viewport::ecad::outdated_json(ctx.app.viewport.ecad_outdated());
    crate::viewport::ecad::keys_json(&mut state, ctx.egui);
    Ok(Outcome::Done(state))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "ecad_action", group: "ecad", doc: "Run one of eCAD's own actions by its id on the active eCAD workbench's editor (Diagram, PCB, Symbol, Pads) — the zoom, selection (delete, duplicate, cancel) and tool actions the row does not carry as well as the ones it does — gated by eCAD's own offered and enabled, as its key or toolbar button would be. Refused, naming the id and listing what is offered, when no eCAD workbench is active or the action is not offered or not enabled; eCAD's own undo and redo are refused by name, since the document keeps one undo history (`undo` / `redo`). The edit reaches the document on the next frame; read it back with `ecad_state`.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<EcadActionArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(ecad_action) },
    CommandSpec { name: "ecad_state", group: "ecad", doc: "The active eCAD workbench's editor, as `__brepEcad` publishes it: target (null when no eCAD workbench is active), inStep (the editor's document is exactly its block of the document), the history revision, pan and zoom, and for the sheet editors the view, tool, selection, components, wires with their terminals, labels, the Connectivity panel's nets and its electrical rule check (erc: run, findings, noConnects, powerFlags) and board (counts, each placement's pads on the board, track paths, the selected pad and the selected silkscreen line); for Symbol its pins, for Pads its pads, silkscreen and selection; and status, the status strip's text with the pointer's place in micrometres and the hit key under it; and outdated, the sheet's components placed from a part whose saved version changed since (reference and part), as the Update parts card lists them.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(ecad_state) },
];
