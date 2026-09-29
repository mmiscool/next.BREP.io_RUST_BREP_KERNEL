//! Whether a popup or menu was up when this frame's input arrived.
//!
//! `BrepApp::handle_shortcuts` takes Escape before any panel or the viewport
//! draws, and egui's `Popup` closes on `key_pressed(Escape)` — which a consumed
//! key never is. So every popup in the app ignored Escape: the workbench
//! switcher kept all 7 of its entries, and a PMI chip's right-click menu kept
//! its 4 through two presses while the first one cleared the selection
//! underneath it, leaving a menu open for an object no longer selected.
//!
//! The shell cannot ask egui "is a popup open?" at that point in the frame.
//! There are two kinds of open state, and neither answers early enough:
//! [`egui::Popup::is_any_open`] sees only the popups whose state egui's memory
//! keeps (the switcher, a `context_menu`), not the `open_bool` menus of
//! `column_tree::action_menu` (every tree row, the PMI chip, the sheet paper);
//! and [`egui::Context::any_popup_open`], which does see both, reads THIS pass,
//! where nothing has been drawn yet. So a plugin records it at the end of each
//! pass — after every popup, of either kind, has drawn — and the next frame's
//! shortcut handler reads that.

use eframe::egui;

/// Whether any popup or menu (not a tooltip) was drawn in the previous pass.
/// Registers the recorder on first use; until it has run once, `false`.
pub fn popup_open_last_pass(ctx: &egui::Context) -> bool {
    // A plugin type is registered once; later calls are ignored.
    ctx.add_plugin(PopupWatch);
    ctx.data(|d| d.get_temp::<bool>(watch_id())).unwrap_or(false)
}

/// Whether a host MODAL was up in the previous pass: any `egui::Modal` — the
/// file dialog in every mode (open, save as, import, export, insert component,
/// confirm close, STEP assembly, KiCad), crash recovery, the command palette,
/// the KiCad import and Submit Bug. egui records the top modal layer as each
/// pass ends, so this needs no list of them, and a new one is covered by being
/// a `Modal`. Last pass, like [`popup_open_last_pass`]: the modals draw after
/// the panels and the tile, so nothing this pass has been drawn when the keys
/// are routed. On the one frame a modal first opens it reads `false`.
///
/// A modal owns the keys. Without this, a key pressed under one that has no
/// text field reached the eCAD editor (R turned a part, Delete removed it) and
/// the host's own keys (Ctrl+Z undid the document; Escape cleared the
/// selection and was spent before the modal could close on it).
pub fn modal_open_last_pass(ctx: &egui::Context) -> bool {
    ctx.memory(|memory| memory.top_modal_layer()).is_some()
}

fn watch_id() -> egui::Id {
    egui::Id::new("brep-popup-open-last-pass")
}

/// Records [`egui::Context::any_popup_open`] as the pass ends.
struct PopupWatch;

impl egui::Plugin for PopupWatch {
    fn debug_name(&self) -> &'static str {
        "brep-popup-watch"
    }

    fn on_end_pass(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx();
        let open = ctx.any_popup_open();
        ctx.data_mut(|d| d.insert_temp(watch_id(), open));
    }
}
