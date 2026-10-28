//! The "Simulation" workbench: motion studies of an assembly.
//!
//! A study is NOT feature creation: the `includes` predicate rejects every
//! catalogue entry. An assembly keeps its studies in its own `simulation`
//! block (`brep_simulation`). A motor is one of the assembly's distance or
//! angle mates, run at a constant speed or along a motion profile; every
//! other part follows through the assembly solver on each frame. What the
//! workbench adds is its Simulation pane — the studies, and the selected
//! study's timeline of motors and clips — and the buttons below. They join the
//! row only on an assembly, the one kind of document a study can drive, so
//! no other document's toolbar changes.

use super::{ButtonState, FeatureInfo, Workbench, WorkbenchButton};

/// The Simulation pane's id (claim + registration key).
pub const PANEL_ID: &str = "simulation";

/// The toolbar buttons' ids.
pub const NEW_STUDY_BUTTON_ID: &str = "simulation.new_study";
pub const ADD_MOTOR_BUTTON_ID: &str = "simulation.add_motor";

/// No creatable features: studies are not history features.
fn includes(_feature: &FeatureInfo<'_>) -> bool {
    false
}

/// Whether the open document is an assembly a study can drive.
fn is_assembly(state: &ButtonState) -> bool {
    state.engine.simulation_available()
}

fn add_motor_disabled(state: &ButtonState) -> Option<&'static str> {
    if state.engine.simulation_motor_mates().is_empty() {
        Some("Add a distance or angle mate first: a motor drives one")
    } else {
        None
    }
}

static BUTTONS: &[WorkbenchButton] = &[
    WorkbenchButton {
        id: NEW_STUDY_BUTTON_ID,
        // A timeline gaining a row.
        glyph: "\u{E0D2}",
        ribbon_path: "Home/Simulation/New study", size: crate::workbench::CommandSize::Large, tooltip: "New motion study of this assembly",
        when: Some(is_assembly),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: ADD_MOTOR_BUTTON_ID,
        // A mate with a turning arrow.
        glyph: "\u{E0D3}",
        ribbon_path: "Home/Simulation/Add motor", size: crate::workbench::CommandSize::Large, tooltip: "Add a motor: drive a distance or angle mate at a speed or along a profile",
        when: Some(is_assembly),
        pressed: None,
        disabled: Some(add_motor_disabled),
        caption: None,
        menu: &[],
    },
];

static PANELS: &[&str] = &[PANEL_ID];

pub static SIMULATION: Workbench = Workbench {
    id: "simulation",
    label: "Simulation",
    // A crank turning on a timeline.
    glyph: "\u{E0D1}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
