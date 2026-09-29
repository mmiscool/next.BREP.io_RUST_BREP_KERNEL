//! The eCAD editors, and their actions as workbench buttons.
//!
//! Four workbenches host eCAD (`brep_ecad_egui`): **Diagram** and **PCB** run its
//! sheet-and-board [`Editor`] — on a wiring document and on a schematic
//! document — while **Symbol** and **Pads** run its [`SymbolEditor`] and
//! [`FootprintEditor`]. eCAD keeps every toolbar command as data
//! ([`brep_ecad_egui::Action`]), with predicates over the editor saying whether it is
//! offered, enabled and pressed, and what its button says. The workbench
//! buttons here are those actions: each button asks eCAD's own predicate about
//! the editor it names ([`offered`], [`pressed`], [`disabled`], [`caption`]),
//! and a press runs eCAD's own action ([`dispatch`]). BREP decides only which
//! actions reach a row and what they look like.
//!
//! # Ids
//!
//! A button id leads with its workbench ([`super::button_prefix`]). Diagram
//! and PCB take their buttons from ONE eCAD table, whose ids lead with a group
//! (`view.`, `sheet.`, `board.`), so each puts its own workbench in front —
//! `diagram.sheet.tool.select`, `pcb.sheet.tool.select` — and [`split`] takes
//! it off again. The symbol and pads editors' ids already lead with `symbol.`
//! and `pads.`, which ARE their workbenches' ids, so their buttons are eCAD's
//! ids verbatim. Two workbenches sharing a table is what made the prefix law
//! necessary: see `a_duplicate_id_names_the_first_workbench_s_button_everywhere`.
//!
//! # Which actions reach a row
//!
//! eCAD groups its actions ([`brep_ecad_egui::ActionGroup`]). The rows carry View,
//! Tool, Layer and Context. History (undo, redo) and Zoom (in, out, fit) are
//! not here: BREP's main toolbar owns Undo, Redo and Zoom to fit, as the
//! Drawing workbench's sheet already relies on. Selection is keyboard-only in
//! eCAD's own toolbars too. Of Context, the six actions that only put the tool
//! down ([`PUT_DOWN`]) are not here: the row's Select and Escape do that, and
//! BREP's sketch row already teaches "press the lit tool to put it down". The
//! six copper-layer actions are one MENU button ([`LAYER_MENU_ID`]), because a
//! layer's name depends on the board's layer count and no fixed picture can say
//! it.
//!
//! # When they are offered
//!
//! An eCAD button is offered only while the document HAS editors, its own
//! workbench is the active one, no sketch is being edited, and eCAD offers the
//! action ([`active_target`]). The active-workbench condition keeps them out
//! of All's row: All draws the 3D view, so no editor is on screen under it, and
//! Diagram's and PCB's copies of one action would stand side by side there —
//! the duplicate-id collision in another form. The sketch condition is the same
//! thought for the shared sketch tools: a sketch takes the viewport over, and
//! its Select beside an editor's Select would be two identical hands.

use super::ButtonState;
use brep_ecad_egui::{Action, Editor, FootprintEditor, SymbolEditor};

/// One document's eCAD editors: what every eCAD button reads and every press
/// runs on. The app keeps one per document, beside its engine, so each tab has
/// its own diagram and board and a tab switch keeps them.
pub struct Editors {
    /// The Diagram workbench's editor, on a wiring document.
    pub diagram: Editor,
    /// The PCB workbench's editor, on a schematic document with its board.
    pub pcb: Editor,
    /// The Symbol workbench's editor: the part's schematic symbol.
    pub symbol: SymbolEditor,
    /// The Pads workbench's editor: the part's footprint pads.
    pub pads: FootprintEditor,
    /// A selection a HOST asked an editor to make, held until that editor is
    /// next drawn: `(which editor, the pin or pad NUMBER)`.
    ///
    /// Deferred, not set at the moment of asking, because the ask crosses a
    /// workbench switch. The editor is re-handed its block on the frame it is
    /// drawn (`viewport::ecad`'s pull), and a load CLEARS the selection — so a
    /// selection made now would be wiped by the very switch that was supposed
    /// to show it. [`Self::apply_focus`] runs after the pull instead, which is
    /// where `set_pins` already runs for the same reason.
    focus: Option<(Target, String)>,
}

impl Editors {
    /// Empty editors: a blank wiring diagram, a blank schematic, a new symbol
    /// and an empty footprint. The Diagram and PCB libraries offer the parts the
    /// document's assembly places (`panels::ecad_parts`); no symbols ship with
    /// the editors.
    pub fn new() -> Self {
        let mut diagram = Editor::default();
        diagram.set_document(brep_ecad_core::Document::new(brep_ecad_core::DocumentKind::Wiring));
        let mut pcb = Editor::default();
        pcb.set_document(brep_ecad_core::Document::new(brep_ecad_core::DocumentKind::Schematic));
        Self {
            diagram,
            pcb,
            symbol: SymbolEditor::blank(),
            pads: FootprintEditor::new(Default::default(), None, Vec::new()),
            focus: None,
        }
    }

    /// Ask `target`'s editor to select the pin or pad numbered `number` the
    /// next time it draws. The Qualify panel's jump: a connection point, its
    /// pin and its pad all carry ONE name, so the number is the whole ask.
    pub fn focus(&mut self, target: Target, number: String) {
        self.focus = Some((target, number));
    }

    /// Make the selection [`Self::focus`] asked for, if it was asked of
    /// `target`. Called by the host AFTER the pull, once per frame the editor
    /// is drawn; returns whether anything was selected. A number the editor
    /// does not have clears the ask all the same — a pin the symbol has not
    /// got yet is not a standing request.
    pub fn apply_focus(&mut self, target: Target) -> bool {
        match self.focus.take_if(|(asked, _)| *asked == target) {
            Some((Target::Symbol, number)) => self.symbol.select_pin(&number),
            Some((Target::Pads, number)) => self.pads.select_pad(&number),
            // Diagram and PCB select components, not pins; nothing asks yet.
            _ => false,
        }
    }

    /// Route selection keys to the visible editor even when the pointer is in a dock.
    pub fn selection_key(&mut self, target: Target, delete: bool) {
        match target {
            Target::Diagram | Target::Pcb => {
                let editor = if target == Target::Diagram { &mut self.diagram } else { &mut self.pcb };
                let action = match (editor.view, delete) {
                    (brep_ecad_egui::View::Board, true) => "board.delete",
                    (brep_ecad_egui::View::Board, false) => "board.cancel",
                    (_, true) => "sheet.delete",
                    (_, false) => "sheet.cancel",
                };
                editor.run_action(action);
            }
            Target::Symbol => { self.symbol.run_action(if delete { "symbol.delete" } else { "symbol.tool.select" }); }
            Target::Pads => { self.pads.run_action(if delete { "pads.delete" } else { "pads.tool.select" }); }
        }
    }

}

impl Default for Editors {
    fn default() -> Self {
        Self::new()
    }
}

/// Which of the four editors a button acts on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    Diagram,
    Pcb,
    Symbol,
    Pads,
}

impl Target {
    pub const ALL: [Target; 4] = [Target::Diagram, Target::Pcb, Target::Symbol, Target::Pads];

    /// The workbench that draws this editor. It is also the key of the
    /// document block the editor's document is kept in (`diagram`, `pcb`,
    /// `symbol`, `pads`): the two namings agree by construction.
    pub fn workbench_id(self) -> &'static str {
        match self {
            Target::Diagram => "diagram",
            Target::Pcb => "pcb",
            Target::Symbol => "symbol",
            Target::Pads => "pads",
        }
    }

    /// The editor a workbench id drives, or `None` for a workbench that is not
    /// one of the four. A plain mapping — unlike [`active_target`] it asks
    /// nothing about editors or sketch mode, so the host can ask it at the
    /// moment it must END a sketch because an eCAD workbench became active.
    pub fn of_workbench(id: &str) -> Option<Target> {
        Target::ALL.into_iter().find(|target| target.workbench_id() == id)
    }
}

/// The dock pane that draws the active editor's parts library
/// (`Editor::library_panel`): Diagram's and PCB's.
pub const LIBRARY_PANEL_ID: &str = "ecadLibrary";

/// The dock pane that draws the active editor's inspector — `Editor::inspector`
/// for Diagram and PCB, `properties` for the symbol and pads editors. One pane
/// for all four, as the assembly panes serve both Assembly and Wire harness.
pub const INSPECTOR_PANEL_ID: &str = "ecadInspector";

/// The editor a button id acts on and the eCAD action id it runs there, or
/// `None` for an id that is no eCAD button's. The inverse of how the tables
/// spell their ids: `diagram.` and `pcb.` come off, `symbol.*` and `pads.*`
/// are eCAD's own.
pub fn split(id: &str) -> Option<(Target, &str)> {
    if let Some(action) = id.strip_prefix("diagram.") {
        Some((Target::Diagram, action))
    } else if let Some(action) = id.strip_prefix("pcb.") {
        Some((Target::Pcb, action))
    } else if id.starts_with("symbol.") {
        Some((Target::Symbol, id))
    } else if id.starts_with("pads.") {
        Some((Target::Pads, id))
    } else {
        None
    }
}

/// The buttons an eCAD workbench carries that are BREP's OWN, not eCAD actions —
/// dispatched by the shell's own arms, never by [`dispatch`]. None today. Listed
/// rather than inferred, so an eCAD button whose id is misspelt fails
/// `every_ecad_workbench_button_is_an_ecad_action_or_brep_s_own` instead of
/// passing quietly as one of BREP's.
pub const BREP_OWN: &[&str] = &[];

/// The editor and eCAD action a button id runs, only when the id IS an action in
/// that editor's own table: [`split`] reads the prefix, this also asks eCAD.
pub fn ecad_action(id: &str) -> Option<(Target, &str)> {
    let (target, action) = split(id)?;
    let known = match target {
        Target::Diagram | Target::Pcb => brep_ecad_egui::actions().any(|a| a.id == action),
        Target::Symbol => brep_ecad_egui::symbol_actions().iter().any(|a| a.id == action),
        Target::Pads => brep_ecad_egui::pad_actions().iter().any(|a| a.id == action),
    };
    known.then_some((target, action))
}

/// The copper-layer MENU button's id. Not an eCAD action: its entries are
/// (`board.layer.1` .. `board.layer.6`).
pub const LAYER_MENU_ID: &str = "pcb.board.layer";

/// The six Context actions that only abandon the gesture in progress and
/// return to Select — five run `Editor::cancel`, the route one drops the draft
/// and selects `BoardTool::Select`. The row's Select button and Escape
/// (`sheet.cancel`, `board.cancel`) already do that, so they are not buttons.
pub const PUT_DOWN: &[&str] = &[
    "sheet.place.done",
    "sheet.wire.done",
    "sheet.label.done",
    "sheet.move.cancel",
    "sheet.junction.done",
    "board.route.done",
];

/// Why eCAD disables an action it offers, for the few that it ever does. eCAD
/// answers `enabled` with a bare bool, so the words are BREP's, one per action,
/// read from the predicate it declares.
const DISABLED_BECAUSE: &[(&str, &str)] = &[
    ("view.schematic", "the autorouter is running"),
    ("view.board", "the autorouter is running"),
    ("sheet.wire.finish", "no wire is being drawn"),
    ("board.route.via", "a via needs a track being drawn on a board with more than one copper layer"),
    ("board.route.finish", "no track is being drawn"),
    ("board.zone.finish", "a zone needs three corners before it can be closed"),
    ("board.zones.fill", "the board has no zones"),
];

/// The words for an action eCAD disables with no entry in
/// [`DISABLED_BECAUSE`] — a table gap, which `every_disabled_action_says_why`
/// fails on rather than letting this reach a user.
const DISABLED_UNEXPLAINED: &str = "the editor does not allow it right now";

/// The editor the active workbench draws, if it is one of the four and
/// something could draw it: the document has editors and no sketch is being
/// edited (a sketch takes the viewport over). See the module doc for why each
/// condition is there.
///
/// The stored workbench id is read directly, not through `resolve`: `resolve`
/// answers only for a workbench in the dropdown, and these four are declared
/// before they are registered ([`super::UNREGISTERED`]). The two readings
/// agree on every registered id.
pub fn active_target(state: &ButtonState) -> Option<Target> {
    state.ecad?;
    if state.engine.sketch_mode() {
        return None;
    }
    Target::of_workbench(&state.engine.settings.workbench)
}

/// What eCAD says about one of its actions on one of its editors.
struct Reading {
    offered: bool,
    enabled: bool,
    pressed: Option<bool>,
    caption: String,
}

fn read_in<T: 'static>(
    mut table: impl Iterator<Item = &'static Action<T>>,
    editor: &T,
    action: &str,
) -> Option<Reading> {
    let action = table.find(|a| a.id == action)?;
    Some(Reading {
        offered: action.offered(editor),
        enabled: action.enabled(editor),
        pressed: action.pressed(editor),
        caption: action.caption(editor),
    })
}

/// eCAD's reading of `action` on `target`'s editor in `editors`.
fn read(editors: &Editors, target: Target, action: &str) -> Option<Reading> {
    match target {
        Target::Diagram => read_in(brep_ecad_egui::actions(), &editors.diagram, action),
        Target::Pcb => read_in(brep_ecad_egui::actions(), &editors.pcb, action),
        Target::Symbol => read_in(brep_ecad_egui::symbol_actions().iter(), &editors.symbol, action),
        Target::Pads => read_in(brep_ecad_egui::pad_actions().iter(), &editors.pads, action),
    }
}

/// eCAD's reading of the action button `id` names, but only while that
/// button's editor is the one on screen ([`active_target`]).
fn reading(state: &ButtonState, id: &str) -> Option<Reading> {
    let (target, action) = split(id)?;
    if active_target(state) != Some(target) {
        return None;
    }
    read(state.ecad?, target, action)
}

/// Whether the eCAD button `id` is offered: its editor is on screen and eCAD
/// offers the action.
pub fn offered(state: &ButtonState, id: &str) -> bool {
    reading(state, id).is_some_and(|r| r.offered)
}

/// Whether the eCAD button `id` draws pressed: eCAD's own toggle state.
pub fn pressed(state: &ButtonState, id: &str) -> bool {
    reading(state, id).is_some_and(|r| r.pressed == Some(true))
}

/// Why the eCAD button `id` cannot act now, or `None` when eCAD enables it.
pub fn disabled(state: &ButtonState, id: &str) -> Option<&'static str> {
    let reading = reading(state, id)?;
    if reading.enabled {
        return None;
    }
    let action = split(id)?.1;
    Some(
        DISABLED_BECAUSE
            .iter()
            .find(|(known, _)| *known == action)
            .map_or(DISABLED_UNEXPLAINED, |(_, why)| *why),
    )
}

/// The eCAD button `id`'s live caption: eCAD's own button text, which for some
/// actions describes state (`Bend: vertical first`, a layer's `B.Cu`).
pub fn caption(state: &ButtonState, id: &str) -> String {
    reading(state, id).map_or_else(
        || super::button_by_declared_id(id).map_or_else(String::new, |b| b.tooltip.to_string()),
        |r| r.caption,
    )
}

/// The layer menu's live caption: the menu's own words and the layer it has
/// on, so the closed menu still says where a track will go.
pub fn layer_menu_caption(state: &ButtonState) -> String {
    let on = super::pcb::LAYERS
        .iter()
        .find(|entry| entry.offered(state) && entry.is_pressed(state))
        .map(|entry| entry.label(state));
    match on {
        Some(layer) => format!("Copper layer: {layer}"),
        None => "Copper layer".to_string(),
    }
}

/// Run the eCAD action the button `id` names on its editor, as a click on
/// eCAD's own button does. `None` when `id` is no declared button that IS an
/// eCAD action — a menu is not one (its entries are), and neither is a button
/// of BREP's own on an eCAD workbench ([`BREP_OWN`]), which the shell's own
/// arms run. Without that, such a button would reach eCAD's `run_action`,
/// read `false`, and be reported handled: a press that vanishes. Otherwise
/// eCAD's answer, `false` when the action is not offered or not enabled now.
pub fn dispatch(editors: &mut Editors, id: &str) -> Option<bool> {
    dispatch_in(&super::declared_workbenches(), editors, id)
}

/// [`dispatch`] over any registry.
fn dispatch_in(registry: &[&'static super::Workbench], editors: &mut Editors, id: &str) -> Option<bool> {
    let button = super::button_in(registry, id)?;
    if !button.menu.is_empty() {
        return None;
    }
    let (target, action) = ecad_action(id)?;
    Some(match target {
        Target::Diagram => editors.diagram.run_action(action),
        Target::Pcb => editors.pcb.run_action(action),
        Target::Symbol => editors.symbol.run_action(action),
        Target::Pads => editors.pads.run_action(action),
    })
}

/// One eCAD action as a workbench button. `$id` is the BUTTON id — eCAD's
/// action id behind its workbench's prefix, see [`split`] — and every
/// predicate is eCAD's own, asked through this module. `toggle` is an action
/// eCAD reports a pressed state for (a tool, a view, a layer); `command` one it
/// does not. The tooltip is eCAD's label, word for word — the tests hold it.
macro_rules! ecad_button {
    ($id:literal, $glyph:literal, $tooltip:literal, toggle) => {
        $crate::workbench::WorkbenchButton {
            id: $id,
            glyph: $glyph,
            tooltip: $tooltip,
            when: Some(|s| $crate::workbench::ecad::offered(s, $id)),
            pressed: Some(|s| $crate::workbench::ecad::pressed(s, $id)),
            disabled: Some(|s| $crate::workbench::ecad::disabled(s, $id)),
            caption: Some(|s| $crate::workbench::ecad::caption(s, $id)),
            menu: &[],
        }
    };
    ($id:literal, $glyph:literal, $tooltip:literal, command) => {
        $crate::workbench::WorkbenchButton {
            id: $id,
            glyph: $glyph,
            tooltip: $tooltip,
            when: Some(|s| $crate::workbench::ecad::offered(s, $id)),
            pressed: None,
            disabled: Some(|s| $crate::workbench::ecad::disabled(s, $id)),
            caption: Some(|s| $crate::workbench::ecad::caption(s, $id)),
            menu: &[],
        }
    };
}
pub(crate) use ecad_button;

/// A workbench whose feature filter offers no features: eCAD's documents are
/// not history features, as PMI's annotations and Drawing's sheets are not.
pub(crate) fn no_features(_feature: &super::FeatureInfo<'_>) -> bool {
    false
}

