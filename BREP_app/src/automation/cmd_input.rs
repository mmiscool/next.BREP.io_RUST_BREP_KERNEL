//! Input-phase commands: the virtual pointer and keyboard (§6). Each becomes
//! `egui::Event`s in the frame's `RawInput`; `click` / `drag` / `hotkey` are
//! host-side compositions of these.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Handler, NoArgs, Phase};
use crate::automation::pointer::{Button, Modifiers, Pointer, PointerState, WheelUnit};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveArgs {
    /// egui points from the top-left of the app surface
    pub x: f32,
    pub y: f32,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ButtonArgs {
    #[serde(default = "primary")]
    pub button: Button,
}
fn primary() -> Button {
    Button::Primary
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WheelArgs {
    #[serde(default)]
    pub dx: f32,
    pub dy: f32,
    #[serde(default = "point")]
    pub unit: WheelUnit,
}
fn point() -> WheelUnit {
    WheelUnit::Point
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct KeyArgs {
    /// An egui key name: `Enter`, `Escape`, `Tab`, `Backspace`, `Delete`, `A`..`Z`, `0`..`9`, `ArrowUp`, `F1`, …
    pub key: String,
    pub pressed: bool,
    #[serde(default)]
    pub repeat: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TextArgs {
    pub text: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PasteArgs {
    /// The clipboard text, e.g. tab-separated rows copied from a spreadsheet.
    pub text: String,
}

fn empty() -> Value {
    json!({})
}

fn pointer_move(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: MoveArgs = parse_args(args)?;
    p.moved(a.x, a.y, out);
    Ok(empty())
}
fn pointer_down(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: ButtonArgs = parse_args(args)?;
    p.button(a.button, true, out)?;
    Ok(empty())
}
fn pointer_up(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: ButtonArgs = parse_args(args)?;
    p.button(a.button, false, out)?;
    Ok(empty())
}
fn pointer_gone(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let _: NoArgs = parse_args(args)?;
    p.gone(out);
    Ok(empty())
}
fn wheel(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: WheelArgs = parse_args(args)?;
    p.wheel(a.dx, a.dy, a.unit, out);
    Ok(empty())
}
fn modifiers_set(p: &mut Pointer, args: Value, _out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let m: Modifiers = parse_args(args)?;
    p.set_modifiers(m);
    Ok(empty())
}
fn key(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: KeyArgs = parse_args(args)?;
    p.key(&a.key, a.pressed, a.repeat, out)?;
    Ok(empty())
}
fn text(p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: TextArgs = parse_args(args)?;
    p.text(&a.text, out);
    Ok(empty())
}
/// A clipboard paste, as the platform delivers Ctrl+V / Cmd+V: one
/// `Event::Paste` carrying the whole text. `text` cannot stand in for it, since
/// that is typing, and a widget that treats a paste as a block (the family
/// table) never sees one.
fn paste(_p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: PasteArgs = parse_args(args)?;
    out.push(egui::Event::Paste(a.text));
    Ok(empty())
}
#[derive(serde::Serialize, schemars::JsonSchema)]
pub struct ClickGap {
    /// Whether egui's click history was reset: false when no injected click
    /// was inside the multi-click window, or a button is held.
    pub reset: bool,
}

fn click_gap(p: &mut Pointer, args: Value, _out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let _: NoArgs = parse_args(args)?;
    let reset = p.click_gap();
    serde_json::to_value(ClickGap { reset }).map_err(|e| e.to_string())
}
fn pointer_state(p: &mut Pointer, args: Value, _out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let _: NoArgs = parse_args(args)?;
    serde_json::to_value(p.state()).map_err(|e| e.to_string())
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ActivateArgs {
    /// A published widget key, `panel/key`, as `hit_rects` lists it.
    pub key: String,
}

#[derive(serde::Serialize, schemars::JsonSchema)]
pub struct Activated {
    pub key: String,
    /// The widget's egui id (its AccessKit node id), in decimal: a u64 is
    /// past a JavaScript number's exact range.
    pub id: String,
    /// The key's rect in the last pass, `[x, y, w, h]` egui points.
    pub rect: [f32; 4],
    /// How the rect was matched to the widget: `exact` (the widget's own
    /// rect) or `contains` (the smallest clickable widget at its centre).
    pub matched: crate::automation::actions::Matched,
}

/// Click a published widget by its id (`automation::actions`): no pointer.
fn activate_widget(_p: &mut Pointer, args: Value, out: &mut Vec<egui::Event>) -> Result<Value, String> {
    let a: ActivateArgs = parse_args(args)?;
    let rect = crate::automation::registry::lock().hit_rects(Some(&a.key)).get(&a.key).copied().ok_or_else(|| {
        format!("widget_not_found: no widget published `{}` in the last pass (`hit_rects` lists the keys; open its panel first)", a.key)
    })?;
    let nodes = crate::automation::actions::nodes();
    let (node, matched) = crate::automation::actions::resolve(&nodes, rect).ok_or_else(|| {
        let nearest = nodes.iter().min_by(|x, y| {
            let d = |n: &crate::automation::actions::Node| (n.rect[0] - rect[0]).abs() + (n.rect[1] - rect[1]).abs();
            d(x).total_cmp(&d(y))
        });
        format!(
            "widget_not_actionable: `{}` has a rect {rect:?} but no enabled clickable widget there (a custom-drawn or disabled control); use `click_widget`. The last pass had {} clickable widgets{}",
            a.key,
            nodes.len(),
            nearest.map(|n| format!(", the nearest at {:?}", n.rect)).unwrap_or_default()
        )
    })?;
    out.push(egui::Event::AccessKitActionRequest(egui::accesskit::ActionRequest {
        action: egui::accesskit::Action::Click,
        target_tree: egui::accesskit::TreeId::ROOT,
        target_node: node.id,
        data: None,
    }));
    serde_json::to_value(Activated { key: a.key, id: node.id.0.to_string(), rect, matched }).map_err(|e| e.to_string())
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "pointer_move", group: "pointer", doc: "Move the virtual pointer to (x, y) in egui points. Emits PointerMoved; hover, drags and picks follow from it.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<MoveArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(pointer_move) },
    CommandSpec { name: "pointer_down", group: "pointer", doc: "Press a pointer button at the current position (default primary). Errors if the pointer has never been moved.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<ButtonArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(pointer_down) },
    CommandSpec { name: "pointer_up", group: "pointer", doc: "Release a pointer button at the current position.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<ButtonArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(pointer_up) },
    CommandSpec { name: "pointer_gone", group: "pointer", doc: "Take the pointer off the surface (PointerGone): clears hover state and released buttons.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<NoArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(pointer_gone) },
    CommandSpec { name: "wheel", group: "pointer", doc: "Scroll at the current pointer position. dy < 0 zooms the viewport in; over a panel it scrolls. Raw MouseWheel, unit point/line/page.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<WheelArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(wheel) },
    CommandSpec { name: "modifiers_set", group: "pointer", doc: "Set the held modifier keys for every following pointer and key event until changed.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<Modifiers>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(modifiers_set) },
    CommandSpec { name: "key", group: "keyboard", doc: "Press (pressed=true) or release a key by egui name, with the held modifiers.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<KeyArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(key) },
    CommandSpec { name: "text", group: "keyboard", doc: "Type text into the focused widget (an egui Text event). Click a field first.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<TextArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(text) },
    CommandSpec { name: "paste", group: "keyboard", doc: "Paste text into the focused widget, as Ctrl+V delivers it (an egui Paste event, the whole text at once). A spreadsheet's tab-separated rows pasted into a table land as a block.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<PasteArgs>, result_schema: crate::automation::command::empty_result_schema, handler: Handler::Input(paste) },
    CommandSpec { name: "click_gap", group: "pointer", doc: "Make the next click the FIRST of a new sequence: when an injected click (a release within 6 pt of its press, down at most 0.8 s; a drag's release is none) landed less than 0.6 s ago (egui's triple-click window, twice the double's 0.3 s), egui's pointer state is replaced with a fresh one, which forgets its click history. egui's clock is NOT moved, so the app's timers (toasts, tooltips, the hover dwell) see nothing; the pointer has no position for that one frame, until the next move. Nothing happens while a button is held. egui counts clicks by time and checks the place only against the last click, so without it a click on another widget a few frames earlier turns a double click into a triple. `click` sends it before it moves, `click_widget` before it reads the widget's rect.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<ClickGap>, handler: Handler::Input(click_gap) },
    CommandSpec { name: "activate_widget", group: "widgets", doc: "Click a published widget (`panel/key`, as `hit_rects` lists it) by its id, not with the pointer: the app asks egui to click the widget whose rect the key published (an AccessKit Click action, as a screen reader sends). Nothing covering the widget, no scroll position and no earlier click can send it elsewhere, and the pointer does not move (no hover). Takes effect in the next pass, as a pointer click does; read the result after `wait_frames`. A key with no enabled clickable widget under its rect (a custom-drawn control) answers `widget_not_actionable`: click it with `click_widget`.", phase: Phase::Input, annotations: Annotations::INPUT, args_schema: schema_of::<ActivateArgs>, result_schema: schema_of::<Activated>, handler: Handler::Input(activate_widget) },
    CommandSpec { name: "pointer_state", group: "pointer", doc: "The virtual pointer: position, held buttons, modifiers.", phase: Phase::Input, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<PointerState>, handler: Handler::Input(pointer_state) },
];
