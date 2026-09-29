//! The "All" workbench: shows every feature and every toolbar icon.
//!
//! Its `includes` predicate accepts every catalogue entry, and its `buttons`
//! slice is INTENTIONALLY EMPTY — "All"'s toolbar buttons are the DERIVED UNION
//! of every other workbench's buttons (see [`super::workbench_buttons`]), so this
//! file never hand-maintains a button list that would rot as workbenches are
//! added. Adding a new workbench file automatically grows "All".

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// "All" owns every feature — no filtering (classifies off nothing).
fn includes(_feature: &FeatureInfo<'_>) -> bool {
    true
}

/// No hand-maintained buttons: "All" derives its buttons as the union of the
/// other workbenches (see [`super::workbench_buttons`]).
static BUTTONS: &[WorkbenchButton] = &[];

/// No panel claims (v1).
static PANELS: &[&str] = &[];

pub static ALL: Workbench = Workbench {
    id: "all",
    label: "All",
    glyph: "\u{E060}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
