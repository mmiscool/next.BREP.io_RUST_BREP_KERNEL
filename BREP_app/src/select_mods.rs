//! Selection modifiers, one meaning on every platform (multi-select rework,
//! owner 2026-11-26).
//!
//! * The PRIMARY modifier is Cmd on macOS and Ctrl on Windows/Linux. egui's
//!   `Modifiers::command` is not enough on its own: the native (egui-winit)
//!   runner sets it to Cmd on macOS, but eframe's WEB runner sets it to
//!   `ctrl || meta` on every OS, so a Mac browser's Ctrl+click would read as a
//!   primary click. On macOS the primary modifier is therefore `mac_cmd`.
//! * On macOS a Ctrl+click is the secondary (right) click, and a Ctrl+drag is
//!   a right drag (pan). `MacSecondary::rewrite` turns those button events
//!   into secondary-button events in the raw input, once, before any widget
//!   sees them — so every context menu, `secondary_clicked()` and
//!   `dragged_by(Secondary)` path behaves as it does for a real right button,
//!   natively and in the browser build, and Ctrl never adds to a selection.
//! * Shift means "more": in the 3D view Shift+click adds/toggles like the
//!   primary modifier; in ordered lists (feature tree, assembly tree, BOM)
//!   Shift+click selects the RANGE from the anchor row (`range`).

/// Whether the PRIMARY modifier (Cmd on macOS, Ctrl elsewhere) is held.
pub fn primary(os: egui::os::OperatingSystem, mods: egui::Modifiers) -> bool {
    if os == egui::os::OperatingSystem::Mac {
        mods.mac_cmd
    } else {
        mods.ctrl || mods.command
    }
}

/// [`primary`] for the context's OS.
pub fn primary_in(ctx: &egui::Context) -> bool {
    let os = ctx.os();
    ctx.input(|i| primary(os, i.modifiers))
}

/// Whether a click adds to (toggles in) the 3D selection: the primary
/// modifier or Shift.
pub fn additive(os: egui::os::OperatingSystem, mods: egui::Modifiers) -> bool {
    primary(os, mods) || mods.shift
}

/// How an ordered list row click applies (feature tree, assembly tree, BOM).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowClick {
    /// Plain click: this row alone.
    Replace,
    /// Primary+click: toggle this row in the selection.
    Toggle,
    /// Shift+click: the range from the anchor to this row, alone.
    Range,
    /// Primary+Shift+click: add the range from the anchor to this row.
    AddRange,
}

impl RowClick {
    pub fn of(os: egui::os::OperatingSystem, mods: egui::Modifiers) -> Self {
        match (primary(os, mods), mods.shift) {
            (false, false) => Self::Replace,
            (true, false) => Self::Toggle,
            (false, true) => Self::Range,
            (true, true) => Self::AddRange,
        }
    }

    pub fn in_ctx(ctx: &egui::Context) -> Self {
        let os = ctx.os();
        ctx.input(|i| Self::of(os, i.modifiers))
    }

    pub fn is_range(self) -> bool {
        matches!(self, Self::Range | Self::AddRange)
    }

    /// Whether this click sets the anchor (plain and primary clicks do; a
    /// range click keeps the anchor it ran from).
    pub fn sets_anchor(self) -> bool {
        !self.is_range()
    }
}

/// The rows from `anchor` to `clicked` inclusive, in display order, or `None`
/// when either is not in `rows` (the anchor row is gone: the caller selects
/// the clicked row alone).
pub fn range<'a, T: PartialEq>(rows: &'a [T], anchor: &T, clicked: &T) -> Option<&'a [T]> {
    let a = rows.iter().position(|r| r == anchor)?;
    let b = rows.iter().position(|r| r == clicked)?;
    Some(&rows[a.min(b)..=a.max(b)])
}

/// The macOS Ctrl+click → secondary click rewrite (see the module docs). One
/// per app: it remembers which presses it turned so their releases match even
/// when Ctrl is let go first.
#[derive(Debug, Default)]
pub struct MacSecondary {
    turned: bool,
}

impl MacSecondary {
    /// Rewrite this frame's `raw` input in place. A no-op off macOS.
    pub fn rewrite(&mut self, os: egui::os::OperatingSystem, raw: &mut egui::RawInput) {
        if os != egui::os::OperatingSystem::Mac {
            self.turned = false;
            return;
        }
        for event in raw.events.iter_mut() {
            if let egui::Event::PointerButton { button, pressed, modifiers, .. } = event {
                if *button != egui::PointerButton::Primary {
                    continue;
                }
                if *pressed {
                    if modifiers.ctrl && !modifiers.mac_cmd {
                        *button = egui::PointerButton::Secondary;
                        self.turned = true;
                    }
                } else if std::mem::take(&mut self.turned) {
                    *button = egui::PointerButton::Secondary;
                }
            }
        }
    }
}

/// Apply [`MacSecondary::rewrite`] to a frame's raw input, keeping its state
/// in the context (one per window). Every host calls it after its own input
/// is in `raw`: the app's `raw_input_hook` and the headless MCP host.
pub fn rewrite_raw(ctx: &egui::Context, raw: &mut egui::RawInput) {
    let id = egui::Id::new("brep-mac-secondary-turned");
    let os = ctx.os();
    let turned = ctx.data(|d| d.get_temp::<bool>(id)).unwrap_or(false);
    let mut rewrite = MacSecondary { turned };
    rewrite.rewrite(os, raw);
    if rewrite.turned != turned {
        ctx.data_mut(|d| d.insert_temp(id, rewrite.turned));
    }
}

