//! The "PMI" (Product & Manufacturing Information) workbench — the mode in
//! which PMI views are captured and annotated.
//!
//! PMI is NOT feature creation: the `includes` predicate rejects every
//! catalogue entry (the Add-feature palette is empty here), and the history
//! stays fully editable as everywhere. What the workbench adds is its PMI
//! panel — the view tree with each view's annotations — the **Capture view**
//! button, the actions strip's **Annotations** group, and the context bar's
//! annotation offers (gated on an active view). The workbench
//! IS the editing mode: entering it remembers the modeling camera / visibility
//! / wireframe, activating a view applies that view's, and leaving restores
//! them (`app.rs` drives the engine's `pmi_enter_workbench` /
//! `pmi_leave_workbench` off the panel's visibility).
//!
//! Drawing sheets — which PLACE the views captured here — are the Drawing
//! workbench's (`workbench/drawing.rs`), not this one's.
//!
//! The one-button-per-annotation-type group is NOT here: like the Constraints
//! group, it is derived from the kernel's table and lives on the workbench
//! actions strip (`panels/workbench_toolbar.rs`), shown wherever this panel is.

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// The PMI panel's id (claim + registration key).
pub const PANEL_ID: &str = "pmi";

/// The Capture-view toolbar button id.
pub const CAPTURE_BUTTON_ID: &str = "pmi.capture_view";

/// No creatable features: annotations are not history features.
fn includes(_feature: &FeatureInfo<'_>) -> bool {
    false
}

static BUTTONS: &[WorkbenchButton] = &[WorkbenchButton {
    id: CAPTURE_BUTTON_ID,
    // A viewfinder round a cube — a view is a captured camera. Not the page
    // (U+1F5CE) it used to draw: that is the Note annotation's icon, and the
    // strip puts the two side by side.
    glyph: "\u{E076}",
    tooltip: "Capture a PMI view (camera + visibility) and start annotating it",
    when: None,
    pressed: None,
    disabled: None,
    caption: None,
    menu: &[],
}];

static PANELS: &[&str] = &[PANEL_ID];

pub static PMI: Workbench = Workbench {
    id: "pmi",
    label: "PMI",
    glyph: "\u{E065}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
