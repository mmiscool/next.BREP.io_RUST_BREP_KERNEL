//! The "Instructions" workbench: manufacturing (assembly) instructions made
//! from an assembly, step by step, for the shop floor and for customers.
//!
//! Instructions are NOT feature creation: the `includes` predicate rejects
//! every catalogue entry. An instructions document places its target assembly
//! as one grounded component and keeps the authored steps in its own
//! `instructions` block (`brep_instructions`); each step shows through its
//! own PMI view. What the workbench adds is its Instructions pane — the
//! step tree and the selected step's editor — its Player
//! pane, which plays the steps in shop or customer mode, and the four
//! toolbar buttons below. The first three are always on the row; one that
//! cannot act on the open document is greyed with the reason, so the way in
//! stays on screen. Play is no way in: it joins the row only on instructions.

use super::{ButtonState, FeatureInfo, Workbench, WorkbenchButton};

/// The Instructions pane's id (claim + registration key).
pub const PANEL_ID: &str = "instructions";
/// The Player pane's id: the in-app player.
pub const PLAYER_PANEL_ID: &str = "instructionsPlayer";

/// The toolbar buttons' ids.
pub const NEW_BUTTON_ID: &str = "instructions.new";
pub const WIZARD_BUTTON_ID: &str = "instructions.wizard";
pub const NEW_STEP_BUTTON_ID: &str = "instructions.new_step";
pub const PLAY_BUTTON_ID: &str = "instructions.play";

/// No creatable features: steps are not history features.
fn includes(_feature: &FeatureInfo<'_>) -> bool {
    false
}

/// Whether the open document places parts (an assembly to describe).
fn places_parts(state: &ButtonState) -> bool {
    (0..state.engine.history_len())
        .any(|index| state.engine.feature_type_at(index).as_deref().is_some_and(is_placement))
}

fn is_placement(kind: &str) -> bool {
    matches!(kind, "ACOMP" | "ASSEMBLY COMPONENT")
}

fn new_disabled(state: &ButtonState) -> Option<&'static str> {
    if state.engine.is_instructions() {
        Some("These are instructions already: open the assembly they describe")
    } else if !places_parts(state) {
        Some("Open an assembly: instructions describe how one is put together")
    } else {
        None
    }
}

fn is_instructions(state: &ButtonState) -> bool {
    state.engine.is_instructions()
}

fn instructions_disabled(state: &ButtonState) -> Option<&'static str> {
    (!state.engine.is_instructions()).then_some("Open or make instructions first (New from assembly)")
}

static BUTTONS: &[WorkbenchButton] = &[
    WorkbenchButton {
        id: NEW_BUTTON_ID,
        // An assembly handing over to a numbered page.
        glyph: "\u{E0CD}",
        ribbon_path: "Home/Instructions/New from assembly", size: crate::workbench::CommandSize::Large, tooltip: "New instructions from this assembly, in a new tab, with the wizard's steps",
        when: None,
        pressed: None,
        disabled: Some(new_disabled),
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: WIZARD_BUTTON_ID,
        // A wand over a list.
        glyph: "\u{E0CE}",
        ribbon_path: "Home/Instructions/Wizard", size: crate::workbench::CommandSize::Large, tooltip: "Wizard: propose the steps again (sub-assemblies first, then mate order, fasteners last)",
        when: None,
        pressed: None,
        disabled: Some(instructions_disabled),
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: NEW_STEP_BUTTON_ID,
        // A list gaining a row.
        glyph: "\u{E0CF}",
        ribbon_path: "Home/Instructions/New step", size: crate::workbench::CommandSize::Large, tooltip: "New step after the selected one; move parts into it from other steps",
        when: None,
        pressed: None,
        disabled: Some(instructions_disabled),
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: PLAY_BUTTON_ID,
        // A play triangle on a page.
        glyph: "\u{E0D0}",
        ribbon_path: "Home/Instructions/Play", size: crate::workbench::CommandSize::Large, tooltip: "Play the instructions step by step, in shop or customer mode",
        when: Some(is_instructions),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
];

/// The BOM is claimed too: its rows are what the author drags into steps.
static PANELS: &[&str] = &[PANEL_ID, PLAYER_PANEL_ID, super::assembly::BOM_PANEL_ID];

pub static INSTRUCTIONS: Workbench = Workbench {
    id: "instructions",
    label: "Instructions",
    // A numbered page over a part.
    glyph: "\u{E0CC}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
