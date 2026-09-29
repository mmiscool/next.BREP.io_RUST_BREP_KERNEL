//! Creatable set: the shared building blocks (sketch `S` / datum `D` / plane
//! `P`) plus the ASSEMBLY COMPONENT (`ACOMP`) — in-context sketches on
//! component faces are an allowed reference use, so S/D/P stay creatable here.
//! Everything else (modeling + sheet metal) is filtered out of the creation
//! UI; the history itself stays fully visible/editable regardless of workbench
//! (standing rule).
//!
//! Claims the two assembly panels (Assembly Structure tree + Assembly
//! Constraints), which therefore show ONLY under Assembly (and "All").

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// The Assembly Structure tree panel's id (claim + registration key).
/// The Assembly Constraints panel's id (the requirements-§5 record id).
pub const CONSTRAINTS_PANEL_ID: &str = "assemblyConstraints";
/// The BOM panel's id (the columned parts list).
pub const BOM_PANEL_ID: &str = "assemblyBom";

/// Datum / Plane / Sketch / Component (`ACOMP`).
fn includes(feature: &FeatureInfo<'_>) -> bool {
    matches!(feature.type_code, "S" | "D" | "P" | "ACOMP")
}

/// Add Component's id — named because the shell arm that answers for it and
/// the PCB row that borrows it both refer to one button, not to a spelling.
pub const ADD_COMPONENT_BUTTON_ID: &str = "assembly.add_component";

/// The assembly toolbar tools. A click surfaces the button id out of the toolbar to the shell:
/// `"assembly.add_component"` opens the insert-component modal (the same
/// `FileAction::InsertComponent` flow the ACOMP palette pick routes through),
/// `"assembly.auto_constrain"` opens the inference window and scans the current placement, and
/// `"assembly.interference"` runs the engine's pairwise check and opens the results window.
/// Like every workbench button, "All" carries them automatically via the deduped union. PCB
/// BORROWS the first of them ([`super::BORROWED`]), which is why this is a named array:
/// [`ADD_COMPONENT`] points INTO it rather than beside it.
static BUTTONS: [WorkbenchButton; 4] = [
    WorkbenchButton {
        id: ADD_COMPONENT_BUTTON_ID,
        // ⊕ (U+2295, circled plus) — ADD a part instance; renders in the
        // bundled DejaVu font (no tofu).
        glyph: "\u{2295}",
        tooltip: "Add component (insert a part from the library or a saved model)",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "assembly.step_parts_library",
        // `menu_book` Material Symbol (base glyph U+1F4DA) — the online
        // step.parts library. `toolbar_button` auto-renders it as a layered
        // colour icon straight from the SVG catalog.
        glyph: "\u{1F4DA}",
        tooltip: "step.parts library — browse & import an online STEP part as a component",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "assembly.auto_constrain",
        // Our own artwork (U+E067): two parts closing on a shared axis with the
        // inference spark. Catalogued SVG, like every picture the app draws.
        glyph: "\u{E067}",
        tooltip: "Auto constraints \u{2014} infer mates (concentric, touch align, coincident) from where the components already sit",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "assembly.interference",
        // ∩ (U+2229, intersection) — the pairwise-INTERSECT tool; renders in the
        // bundled DejaVu font (no tofu).
        glyph: "\u{2229}",
        tooltip: "Interference check (pairwise intersect)",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
];

/// The ADD COMPONENT button ITSELF — the one declaration, lent to the PCB
/// workbench through [`super::BORROWED`]. A borrower shows THIS button, not a
/// copy of it, so the id, the picture, the tooltip and the shell arm that
/// answers for it cannot drift apart.
pub static ADD_COMPONENT: &WorkbenchButton = &BUTTONS[0];

/// The assembly panels this workbench claims (claim-based visibility —
/// unclaimed panels stay visible everywhere; these show only here + All).
static PANELS: &[&str] = &[CONSTRAINTS_PANEL_ID, BOM_PANEL_ID];

pub static ASSEMBLY: Workbench = Workbench {
    id: "assembly",
    label: "Assembly",
    glyph: "\u{E064}",
    includes,
    buttons: &BUTTONS,
    panels: PANELS,
};
