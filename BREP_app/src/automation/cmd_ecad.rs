//! The eCAD workbenches: eCAD's own actions by id, and the editor's state.
//!
//! `workbench_button` presses what the Harness diagram, PCB diagram, PCB,
//! Symbol and Pads rows carry. These reach the rest of eCAD's action tables — zoom, the selection's
//! commands (delete, duplicate, cancel), the actions that only put a tool down
//! — by eCAD's own id, gated by eCAD's own `offered` and `enabled`, as a key or
//! a click on eCAD's toolbar would be. The one group refused is History: eCAD's
//! own undo history would be a second one beside the document's, which is
//! exactly what the host keeps shut (`handle_shortcuts` takes the history keys
//! before any editor sees them), so the command names `undo` instead.
//!
//! eCAD's two view actions, `view.schematic` and `view.board`, are no row's
//! buttons: the schematic and the board are two workbenches (PCB diagram and
//! PCB, `workbench::ecad::BENCHES`), each holding the one editor in its view. So
//! here they SWITCH to the workbench that shows that view, as a `settings_set`
//! of the workbench would, and say so — from either of those two; any other
//! eCAD workbench refuses them by name.
use crate::automation::command::{app_command, parse_args, CommandSpec, Ctx, NoArgs, Outcome};
use crate::workbench::ecad::Target;
use brep_ecad_egui::{Action, ActionGroup};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EcadActionArgs {
    /// eCAD's own action id, e.g. `sheet.delete`, `board.cancel`,
    /// `symbol.tool.pin`. A Harness diagram, PCB diagram or PCB row button's id
    /// (`schematic.sheet.tool.wire`) is accepted too, with its workbench prefix.
    /// `view.schematic` and `view.board` switch to PCB diagram and PCB.
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
    let doc = ctx.app.docs.view_mut();
    let workbench = doc.engine.settings.workbench.clone();
    let Some(target) = Target::of_workbench(&workbench) else {
        return Err(format!(
            "`{}`: no eCAD workbench is active (active: `{}`) — choose diagram, schematic, pcb, symbol or pads",
            a.id,
            crate::workbench::resolve(&workbench).id
        ));
    };
    // The editor in the view its workbench shows, even on the frame the
    // workbench was written (the app pins it once per frame, before the rows).
    doc.ecad.pin_view(&workbench);
    // A row button's id carries its workbench in front; eCAD's does not.
    let id = a.id.strip_prefix(workbench.as_str()).and_then(|rest| rest.strip_prefix('.'));
    let id = match target {
        Target::Pcb => id.unwrap_or(&a.id),
        // The symbol and pads editors' ids already lead with their workbench.
        Target::Symbol | Target::Pads => &a.id,
    };
    // The two views are two workbenches: the view action switches to the one
    // that shows it.
    let view_bench = match id {
        "view.schematic" => Some("schematic"),
        "view.board" => Some("pcb"),
        _ => None,
    };
    if let Some(to) = view_bench {
        if target != Target::Pcb {
            return Err(format!(
                "`{id}` switches between PCB diagram and PCB; the {workbench} editor has no {} view",
                if to == "pcb" { "board" } else { "schematic" }
            ));
        }
        if doc.ecad.pcb.routing() {
            return Err(format!("`{id}` is not enabled now: the autorouter is running"));
        }
        if to != workbench {
            doc.engine.apply_settings_json(&json!({ "workbench": to }).to_string())?;
            doc.ecad.pin_view(to);
        }
        return Ok(Outcome::Done(json!({ "id": id, "workbench": to })));
    }
    let name = workbench.as_str();
    match target {
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
    let doc = ctx.app.docs.view();
    let Some(target) = Target::of_workbench(&doc.engine.settings.workbench) else {
        return Ok(Outcome::Done(json!({ "target": null })));
    };
    let block = crate::viewport::ecad::block_of(target);
    let sync = doc
        .ecad_blocks
        .iter()
        .find(|sync| sync.block() == block)
        .expect("a document keeps one BlockSync per block");
    // The sheet is unreadable when either of its blocks is.
    let sync = match doc.ecad_blocks.iter().find(|s| target == Target::Pcb && s.block() == crate::document::ecad::Block::Diagram) {
        Some(diagram) if diagram.unreadable().is_some() && sync.unreadable().is_none() => diagram,
        _ => sync,
    };
    let mut state = crate::viewport::ecad::state_json(target, &doc.ecad, sync, &doc.engine.history);
    // The workbench itself, not the view the editor was last held in: right
    // after a `settings_set` the frame that pins the view has not run yet.
    state["target"] = json!(crate::workbench::resolve(&doc.engine.settings.workbench).id);
    state["status"] = crate::viewport::ecad::status_json(ctx.app.viewport.ecad_status());
    state["outdated"] = crate::viewport::ecad::outdated_json(ctx.app.viewport.ecad_outdated());
    crate::viewport::ecad::keys_json(&mut state, ctx.egui);
    Ok(Outcome::Done(state))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("ecad", "ecad_action", "Run one of eCAD's own actions by its id on the active eCAD workbench's editor (Harness diagram `diagram`, PCB diagram `schematic`, PCB `pcb`, Symbol, Pads) — the zoom, selection (delete, duplicate, cancel) and tool actions the row does not carry as well as the ones it does — gated by eCAD's own offered and enabled, as its key or toolbar button would be. Refused, naming the id and listing what is offered, when no eCAD workbench is active or the action is not offered or not enabled; eCAD's own undo and redo are refused by name, since the document keeps one undo history (`undo` / `redo`). `view.schematic` and `view.board` switch to the PCB diagram and PCB workbenches, which show those views, and the reply names the workbench. The edit reaches the document on the next frame; read it back with `ecad_state`.", EcadActionArgs, ecad_action, Mutate, MUTATE_NOWAIT, super::domain_output_schema::ecad_action),
    app_command!("ecad", "ecad_state", "The active eCAD workbench's editor, as `__brepEcad` publishes it: target (null when no eCAD workbench is active), inStep (the editor's document is exactly its block of the document), the history revision, pan and zoom, and for the sheet editors the view, tool, selection, components, wires with their terminals, labels, the Connectivity panel's nets and its electrical rule check (erc: run, findings, noConnects, powerFlags) and board (counts, each placement's pads on the board, track paths, the selected pad and the selected silkscreen line); for Symbol its pins, for Pads its pads, silkscreen and selection; and status, the status strip's text with the pointer's place in micrometres and the hit key under it; and outdated, the sheet's components placed from a part whose saved version changed since (reference and part), as the Update parts card lists them.", NoArgs, ecad_state, Read, READ, super::domain_output_schema::ecad_state),
];
