//! Every toolbar command and keyboard shortcut, as data. Each editor's own toolbar draws
//! these and its keyboard handling runs them, so a host that lists an editor's actions
//! and runs them by id offers exactly what that editor's toolbar does.
use super::*;
use egui::{KeyboardShortcut, Modifiers};

/// Where an action sits in the editor's own toolbar.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ActionGroup {
    /// Switching between the schematic and the board.
    View,
    /// Choosing the pointer tool.
    Tool,
    /// Undo and redo.
    History,
    /// Zooming and fitting.
    Zoom,
    /// The copper layer to route on.
    Layer,
    /// The snapping grid: one action per step, of which one is pressed.
    Grid,
    /// Commands for the tool in use, on the toolbar's second row.
    Context,
    /// Commands on the selection or the gesture in progress, run from the keyboard.
    Selection,
}

/// One command of an editor `T`. Its predicates read the editor, so they answer for the
/// current state.
pub struct Action<T> {
    /// Stable identifier, such as `sheet.tool.wire`.
    pub id: &'static str,
    /// Name for menus and tooltips.
    pub label: &'static str,
    pub group: ActionGroup,
    /// Keys that run it while the pointer is over the sheet or board. Shift and Alt
    /// are ignored unless listed.
    pub shortcuts: &'static [KeyboardShortcut],
    offered: fn(&T) -> bool,
    enabled: fn(&T) -> bool,
    pressed: Option<fn(&T) -> bool>,
    caption: Option<fn(&T) -> String>,
    run: fn(&mut T),
}
impl<T> Action<T> {
    /// Whether the editor offers this action now: its view and tool are showing.
    pub fn offered(&self, editor: &T) -> bool {
        (self.offered)(editor)
    }
    /// Whether an offered action would do something now.
    pub fn enabled(&self, editor: &T) -> bool {
        (self.enabled)(editor)
    }
    /// For a toggle or choice, whether it is on; `None` for a plain command.
    pub fn pressed(&self, editor: &T) -> Option<bool> {
        self.pressed.map(|pressed| pressed(editor))
    }
    /// Button text, which for some actions describes the current state, such as
    /// "Bend: vertical first".
    pub fn caption(&self, editor: &T) -> String {
        self.caption
            .map_or_else(|| self.label.to_owned(), |caption| caption(editor))
    }
}

/// Every action the sheet and board editor has, in toolbar order.
pub fn actions() -> impl Iterator<Item = &'static Action<Editor>> {
    SHARED.iter().chain(SHEET).chain(board_view::BOARD_ACTIONS)
}

pub(crate) const fn action<T>(
    id: &'static str,
    label: &'static str,
    group: ActionGroup,
    run: fn(&mut T),
) -> Action<T> {
    Action {
        id,
        label,
        group,
        shortcuts: &[],
        offered: |_| true,
        enabled: |_| true,
        pressed: None,
        caption: None,
        run,
    }
}
impl<T> Action<T> {
    pub(crate) const fn offered_when(mut self, offered: fn(&T) -> bool) -> Self {
        self.offered = offered;
        self
    }
    pub(crate) const fn enabled_when(mut self, enabled: fn(&T) -> bool) -> Self {
        self.enabled = enabled;
        self
    }
    pub(crate) const fn pressed_when(mut self, pressed: fn(&T) -> bool) -> Self {
        self.pressed = Some(pressed);
        self
    }
    pub(crate) const fn captioned(mut self, caption: fn(&T) -> String) -> Self {
        self.caption = Some(caption);
        self
    }
    pub(crate) const fn keys(mut self, shortcuts: &'static [KeyboardShortcut]) -> Self {
        self.shortcuts = shortcuts;
        self
    }
}
/// [`action`] for the sheet and board editor, so a table's closures need no type.
pub(crate) const fn sheet_action(
    id: &'static str,
    label: &'static str,
    group: ActionGroup,
    run: fn(&mut Editor),
) -> Action<Editor> {
    action(id, label, group, run)
}
pub(crate) const fn key(key: Key) -> KeyboardShortcut {
    KeyboardShortcut::new(Modifiers::NONE, key)
}
pub(crate) const fn command(key: Key) -> KeyboardShortcut {
    KeyboardShortcut::new(Modifiers::COMMAND, key)
}

fn sheet(e: &Editor) -> bool {
    e.view == View::Schematic
}
fn sheet_tool(e: &Editor, tool: Tool) -> bool {
    sheet(e) && e.tool == tool
}
/// Tools that draw nets, which a wiring diagram does not have.
fn net_tools(e: &Editor) -> bool {
    sheet(e) && !e.wiring()
}

static SHARED: &[Action<Editor>] = &[
    sheet_action("view.schematic", "Schematic", ActionGroup::View, |e| {
        e.set_view(View::Schematic)
    })
    .offered_when(|e| !e.wiring())
    .enabled_when(|e| !e.routing())
    .pressed_when(|e| e.view == View::Schematic),
    sheet_action("view.board", "PCB", ActionGroup::View, |e| {
        e.set_view(View::Board)
    })
    .offered_when(|e| !e.wiring())
    .enabled_when(|e| !e.routing())
    .pressed_when(|e| e.view == View::Board),
    sheet_action("edit.undo", "Undo", ActionGroup::History, |e| {
        e.step_history(HistoryKey::Undo)
    })
    .keys(&[command(Key::Z)])
    .enabled_when(|e| e.can_step_history(HistoryKey::Undo)),
    sheet_action("edit.redo", "Redo", ActionGroup::History, |e| {
        e.step_history(HistoryKey::Redo)
    })
    .keys(&[
        KeyboardShortcut::new(Modifiers::COMMAND.plus(Modifiers::SHIFT), Key::Z),
        command(Key::Y),
    ])
    .enabled_when(|e| e.can_step_history(HistoryKey::Redo)),
    sheet_action("view.zoom_out", "Zoom −", ActionGroup::Zoom, |e| {
        e.zoom_view(1. / 1.25)
    }),
    sheet_action("view.zoom_in", "Zoom +", ActionGroup::Zoom, |e| {
        e.zoom_view(1.25)
    }),
    sheet_action("view.fit", "Fit", ActionGroup::Zoom, Editor::fit_view)
        .keys(&[key(Key::F)])
        .captioned(|e| if sheet(e) { "Fit sheet" } else { "Fit board" }.into()),
];

static SHEET: &[Action<Editor>] = &[
    sheet_action("sheet.tool.select", "Select", ActionGroup::Tool, |e| {
        e.set_tool(Tool::Select)
    })
    .keys(&[key(Key::V)])
    .offered_when(sheet)
    .pressed_when(|e| e.tool == Tool::Select),
    sheet_action("sheet.tool.wire", "Draw wire", ActionGroup::Tool, |e| {
        e.set_tool(Tool::Wire)
    })
    .keys(&[key(Key::W)])
    .offered_when(net_tools)
    .pressed_when(|e| e.tool == Tool::Wire),
    sheet_action("sheet.tool.junction", "Junction", ActionGroup::Tool, |e| {
        e.set_tool(Tool::Junction)
    })
    .keys(&[key(Key::J)])
    .offered_when(net_tools)
    .pressed_when(|e| e.tool == Tool::Junction),
    sheet_action("sheet.tool.label", "Net label", ActionGroup::Tool, |e| {
        e.set_tool(Tool::Label)
    })
    .keys(&[key(Key::L)])
    .offered_when(net_tools)
    .pressed_when(|e| e.tool == Tool::Label),
    sheet_action("sheet.tool.no_connect", "No-connect", ActionGroup::Tool, |e| {
        e.set_tool(Tool::NoConnect)
    })
    .keys(&[key(Key::Q)])
    .offered_when(net_tools)
    .pressed_when(|e| e.tool == Tool::NoConnect),
    sheet_action("sheet.tool.power_flag", "Power flag", ActionGroup::Tool, |e| {
        e.set_tool(Tool::PowerFlag)
    })
    .offered_when(net_tools)
    .pressed_when(|e| e.tool == Tool::PowerFlag),
    sheet_action("sheet.tool.power", "Power symbol", ActionGroup::Tool, |e| {
        e.set_tool(Tool::Power)
    })
    .keys(&[key(Key::P)])
    .offered_when(net_tools)
    .pressed_when(|e| e.tool == Tool::Power),
    sheet_action(
        "sheet.place.rotate",
        "Rotate 90°",
        ActionGroup::Context,
        Editor::rotate,
    )
    .offered_when(|e| sheet_tool(e, Tool::Place)),
    sheet_action(
        "sheet.place.done",
        "Done placing",
        ActionGroup::Context,
        Editor::cancel,
    )
    .offered_when(|e| sheet_tool(e, Tool::Place)),
    sheet_action(
        "sheet.wire.bend",
        "Bend direction",
        ActionGroup::Context,
        |e| e.vertical_first = !e.vertical_first,
    )
    .keys(&[key(Key::Tab)])
    .offered_when(|e| sheet_tool(e, Tool::Wire))
    .captioned(|e| {
        if e.vertical_first {
            "Bend: vertical first"
        } else {
            "Bend: horizontal first"
        }
        .into()
    }),
    sheet_action(
        "sheet.wire.finish",
        "Finish wire",
        ActionGroup::Context,
        |e| e.wire_start = None,
    )
    .keys(&[key(Key::Enter)])
    .offered_when(|e| sheet_tool(e, Tool::Wire))
    .enabled_when(|e| e.wire_start.is_some()),
    sheet_action(
        "sheet.wire.done",
        "Done wiring",
        ActionGroup::Context,
        Editor::cancel,
    )
    .offered_when(|e| sheet_tool(e, Tool::Wire)),
    sheet_action(
        "sheet.label.done",
        "Done labeling",
        ActionGroup::Context,
        Editor::cancel,
    )
    .offered_when(|e| sheet_tool(e, Tool::Label)),
    sheet_action(
        "sheet.move.cancel",
        "Cancel move",
        ActionGroup::Context,
        Editor::cancel,
    )
    .offered_when(|e| sheet_tool(e, Tool::Move)),
    sheet_action(
        "sheet.junction.done",
        "Done",
        ActionGroup::Context,
        Editor::cancel,
    )
    .offered_when(|e| sheet_tool(e, Tool::Junction)),
    sheet_action(
        "sheet.rotate",
        "Rotate 90°",
        ActionGroup::Selection,
        Editor::rotate,
    )
    .keys(&[key(Key::R)])
    .offered_when(sheet)
    .enabled_when(|e| {
        e.tool == Tool::Place
            || matches!(
                e.selected,
                Some(Selection::Component(_) | Selection::Label(_))
            )
    }),
    sheet_action(
        "sheet.duplicate",
        "Duplicate",
        ActionGroup::Selection,
        Editor::duplicate,
    )
    .keys(&[command(Key::D)])
    .offered_when(sheet)
    .enabled_when(|e| matches!(e.selected, Some(Selection::Component(_)))),
    sheet_action(
        "sheet.delete",
        "Delete selection",
        ActionGroup::Selection,
        Editor::delete,
    )
    .keys(&[key(Key::Delete), key(Key::Backspace)])
    .offered_when(sheet)
    .enabled_when(|e| e.selected.is_some()),
    sheet_action(
        "sheet.cancel",
        "Cancel",
        ActionGroup::Selection,
        Editor::cancel,
    )
    .keys(&[key(Key::Escape)])
    .offered_when(sheet),
];

/// Modifier keys a shortcut asks for; a press matching several shortcuts runs the one
/// asking for the most, so Ctrl+Shift+Z redoes rather than undoes.
fn specificity(modifiers: Modifiers) -> u8 {
    u8::from(modifiers.alt) + u8::from(modifiers.shift) + u8::from(modifiers.command)
}

/// Run `id` on `editor`, as a click on its button does. False when there is no such
/// action, or it is not offered or not enabled now.
pub(crate) fn run_action<T: 'static>(
    editor: &mut T,
    actions: impl Iterator<Item = &'static Action<T>>,
    id: &str,
) -> bool {
    match actions.into_iter().find(|a| a.id == id) {
        Some(a) if a.offered(editor) && a.enabled(editor) => {
            (a.run)(editor);
            true
        }
        _ => false,
    }
}

/// Run the actions whose shortcuts were pressed this frame, most specific first.
pub(crate) fn run_shortcuts<T: 'static>(
    editor: &mut T,
    actions: &[&'static Action<T>],
    ctx: &egui::Context,
) {
    let presses: Vec<(Key, Modifiers)> = ctx.input(|i| {
        i.events
            .iter()
            .filter_map(|event| match event {
                egui::Event::Key {
                    key,
                    pressed: true,
                    modifiers,
                    ..
                } => Some((*key, *modifiers)),
                _ => None,
            })
            .collect()
    });
    for (key, modifiers) in presses {
        let bound = actions
            .iter()
            .copied()
            .filter(|a| a.offered(editor))
            .filter_map(|a| {
                a.shortcuts
                    .iter()
                    .find(|s| s.logical_key == key && modifiers.matches_logically(s.modifiers))
                    .map(|s| (a, specificity(s.modifiers)))
            })
            .max_by_key(|(_, specificity)| *specificity);
        if let Some((action, _)) = bound
            && action.enabled(editor)
        {
            // Handled here, so an open editor window's own history keys do not
            // run it a second time this frame.
            ctx.input_mut(|i| i.consume_key(modifiers, key));
            (action.run)(editor);
        }
    }
}

/// Draw the offered actions of one group as a toolbar's buttons.
pub(crate) fn action_buttons<T: 'static>(
    editor: &mut T,
    actions: impl Iterator<Item = &'static Action<T>>,
    ui: &mut egui::Ui,
    group: ActionGroup,
) {
    let offered: Vec<&'static Action<T>> = actions
        .into_iter()
        .filter(|a| a.group == group && a.offered(editor))
        .collect();
    for action in offered {
        let caption = action.caption(editor);
        let button = match action.pressed(editor) {
            Some(pressed) => egui::Button::selectable(pressed, caption),
            None => egui::Button::new(caption),
        };
        if ui.add_enabled(action.enabled(editor), button).clicked() {
            (action.run)(editor);
        }
    }
}

impl Editor {
    /// Run an action by id, as a click on its button does. Returns false when there is
    /// no such action, or it is not offered or not enabled now.
    pub fn run_action(&mut self, id: &str) -> bool {
        run_action(self, actions(), id)
    }
    pub(crate) fn run_shortcuts(&mut self, ctx: &egui::Context) {
        run_shortcuts(self, &actions().collect::<Vec<_>>(), ctx);
    }
    /// Draw the offered actions of one group as the toolbar's buttons.
    pub(crate) fn action_buttons(&mut self, ui: &mut egui::Ui, group: ActionGroup) {
        action_buttons(self, actions(), ui, group);
    }
    fn step_history(&mut self, key: HistoryKey) {
        match key {
            HistoryKey::Undo => self.undo(),
            HistoryKey::Redo => self.redo(),
        }
    }
    fn can_step_history(&self, key: HistoryKey) -> bool {
        !self.routing()
            && match key {
                HistoryKey::Undo => self.history.can_undo(),
                HistoryKey::Redo => self.history.can_redo(),
            }
    }
    fn zoom_view(&mut self, factor: f32) {
        match self.view {
            View::Schematic => self.zoom_by(factor),
            View::Board => self.zoom_board(factor),
        }
    }
    fn fit_view(&mut self) {
        match self.view {
            View::Schematic => self.fit(self.canvas_size),
            View::Board => self.fit_board(),
        }
    }
}
