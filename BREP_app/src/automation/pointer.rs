//! The stateful virtual pointer and keyboard (§6): position, held buttons and
//! modifiers persist across frames; each input command becomes one or more
//! `egui::Event`s pushed into the frame's `RawInput`.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Button {
    Primary,
    Secondary,
    Middle,
    Extra1,
    Extra2,
}

impl Button {
    pub fn egui(self) -> egui::PointerButton {
        match self {
            Button::Primary => egui::PointerButton::Primary,
            Button::Secondary => egui::PointerButton::Secondary,
            Button::Middle => egui::PointerButton::Middle,
            Button::Extra1 => egui::PointerButton::Extra1,
            Button::Extra2 => egui::PointerButton::Extra2,
        }
    }
    fn index(self) -> usize {
        self as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum WheelUnit {
    Point,
    Line,
    Page,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Modifiers {
    #[serde(default)]
    pub ctrl: bool,
    #[serde(default)]
    pub shift: bool,
    #[serde(default)]
    pub alt: bool,
    /// Cmd on macOS; treated as Ctrl elsewhere.
    #[serde(default)]
    pub command: bool,
}

impl Modifiers {
    pub fn egui(self) -> egui::Modifiers {
        egui::Modifiers {
            alt: self.alt,
            ctrl: self.ctrl || self.command,
            shift: self.shift,
            mac_cmd: self.command,
            command: self.ctrl || self.command,
        }
    }
}

/// The pointer as reported to callers.
#[derive(Debug, Clone, Serialize, schemars::JsonSchema)]
pub struct PointerState {
    /// Position in egui points, `null` when the pointer is not over the surface.
    pub pos: Option<[f32; 2]>,
    pub buttons: Vec<Button>,
    pub modifiers: Modifiers,
}

/// egui's multi-click window: a release less than this after the last click,
/// within `max_click_dist` of it, is a double click, and one less than TWICE
/// this after the click before that is a triple (`InputOptions` default,
/// `egui::input_state::PointerState::begin_pass`). The app keeps the default;
/// `the_multi_click_window_is_egui_s` pins the copy.
pub const MULTI_CLICK_WINDOW: f64 = 0.3;

/// egui's click limits (`InputOptions` defaults): a release is a click only
/// if the pointer stayed within this many points of the press ...
pub const CLICK_DIST: f32 = 6.0;
/// ... and the button was down no longer than this.
pub const CLICK_DURATION: f64 = 0.8;

#[derive(Debug, Clone, Default)]
pub struct Pointer {
    pub pos: Option<egui::Pos2>,
    pub buttons: [bool; 5],
    pub modifiers: Modifiers,
    /// egui's clock for the frame being drained, set by
    /// [`drain_input`](crate::automation::queue::AutomationQueue::drain_input)
    /// before the frame's commands run.
    pub now: f64,
    /// Set by [`click_gap`](Self::click_gap): the queue replaces egui's
    /// `PointerState` with a fresh one after this frame's commands.
    pub reset_clicks: bool,
    /// egui time of the last injected release that egui counts as a CLICK.
    /// A drag's release is not one, and must not make the next click wait.
    released: Option<f64>,
    /// Where and when the held button went down, and whether the pointer has
    /// since left [`CLICK_DIST`] of it: egui's own test for a click.
    press: Option<(egui::Pos2, f64)>,
    moved_far: bool,
}

const ALL_BUTTONS: [Button; 5] = [Button::Primary, Button::Secondary, Button::Middle, Button::Extra1, Button::Extra2];

impl Pointer {
    pub fn state(&self) -> PointerState {
        PointerState {
            pos: self.pos.map(|p| [p.x, p.y]),
            buttons: ALL_BUTTONS.iter().copied().filter(|b| self.buttons[b.index()]).collect(),
            modifiers: self.modifiers,
        }
    }

    pub fn moved(&mut self, x: f32, y: f32, out: &mut Vec<egui::Event>) {
        let pos = egui::pos2(x, y);
        self.pos = Some(pos);
        if let Some((origin, _)) = self.press {
            self.moved_far |= origin.distance(pos) > CLICK_DIST;
        }
        out.push(egui::Event::PointerMoved(pos));
    }

    pub fn button(&mut self, button: Button, pressed: bool, out: &mut Vec<egui::Event>) -> Result<(), String> {
        let pos = self.pos.ok_or("the pointer has no position yet: move it first")?;
        self.buttons[button.index()] = pressed;
        let now = self.now;
        if pressed {
            self.press = Some((pos, now));
            self.moved_far = false;
        } else if let Some((_, down)) = self.press.take() {
            if !self.moved_far && now - down <= CLICK_DURATION {
                self.released = Some(now);
            }
        }
        out.push(egui::Event::PointerButton { pos, button: button.egui(), pressed, modifiers: self.modifiers.egui() });
        Ok(())
    }

    /// Start a NEW click sequence: forget egui's click history when an
    /// injected click is still inside its multi-click window. Returns whether
    /// it did.
    ///
    /// egui counts clicks by time, and checks the place only against the LAST
    /// click, so a click on one widget a few frames (1/60 s each headless)
    /// before a double click on another makes the double a TRIPLE, and a
    /// widget clicked twice by two scripted single clicks reads the second as
    /// a double (ecad-kicad-dialog-folders-2026-09-23).
    ///
    /// The history is egui's `PointerState`, whose click times are private;
    /// the queue swaps in `PointerState::default()` (its options are refreshed
    /// every pass). egui's CLOCK is not touched: the first version jumped it
    /// past the window instead, and the app's own timers saw the jump, so a
    /// toast aged 0.6 s per click (ecad-mcp-click-count-export-wording).
    /// The window is the TRIPLE one, twice the double's: the second click of
    /// a new double looks two clicks back. Nothing is reset while a button is
    /// held (a drag in progress lives in the same state), or when the last
    /// click is already outside the window.
    pub fn click_gap(&mut self) -> bool {
        let recent = self.released.is_some_and(|t| self.now - t < 2.0 * MULTI_CLICK_WINDOW);
        if !recent || self.buttons.iter().any(|&b| b) {
            return false;
        }
        self.released = None;
        self.reset_clicks = true;
        true
    }

    pub fn gone(&mut self, out: &mut Vec<egui::Event>) {
        self.pos = None;
        self.buttons = [false; 5];
        out.push(egui::Event::PointerGone);
    }

    pub fn wheel(&mut self, dx: f32, dy: f32, unit: WheelUnit, out: &mut Vec<egui::Event>) {
        let unit = match unit {
            WheelUnit::Point => egui::MouseWheelUnit::Point,
            WheelUnit::Line => egui::MouseWheelUnit::Line,
            WheelUnit::Page => egui::MouseWheelUnit::Page,
        };
        out.push(egui::Event::MouseWheel {
            unit,
            delta: egui::vec2(dx, dy),
            phase: egui::TouchPhase::Move,
            modifiers: self.modifiers.egui(),
        });
    }

    pub fn set_modifiers(&mut self, m: Modifiers) {
        self.modifiers = m;
    }

    pub fn key(&mut self, name: &str, pressed: bool, repeat: bool, out: &mut Vec<egui::Event>) -> Result<(), String> {
        let key = egui::Key::from_name(name).ok_or_else(|| format!("unknown key `{name}` (egui key names: A..Z, 0..9, Enter, Escape, Tab, Space, Backspace, Delete, ArrowLeft/Right/Up/Down, Home, End, PageUp/Down, F1..F12, Minus, Plus, …)"))?;
        out.push(egui::Event::Key { key, physical_key: None, pressed, repeat, modifiers: self.modifiers.egui() });
        Ok(())
    }

    pub fn text(&mut self, text: &str, out: &mut Vec<egui::Event>) {
        if !text.is_empty() {
            out.push(egui::Event::Text(text.to_string()));
        }
    }
}
