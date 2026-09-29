//! The "Wire harness" workbench: routed wire runs between CONNECTION POINTS
//! over a network of SPLINE paths.
//!
//! Creatable set: the harness authoring geometry — `WP` (a routing waypoint a
//! wire passes through), `SP` (a spline whose end anchors attach to connection
//! points and so becomes a harness segment), plus the shared building blocks
//! `S` / `D` / `P` and `ACOMP` (a harness routes across placed components,
//! whose declared points are its wire ends). Every modeling and sheet-metal
//! feature is filtered out of the creation lists; the history stays fully
//! editable.
//!
//! A part's OWN connection points are not here: they are the `ports` block,
//! which is part data rather than a modelling step (`ports.rs`). They are
//! managed in the **Qualify** pane — which is on screen exactly while the part
//! declares some, so this workbench carries the one button that DECLARES THE
//! FIRST one ([`DECLARE_POINT_BUTTON_ID`]) and thereby opens that pane. It is
//! the only way into the ports block that is neither the KiCad import, nor
//! editing a symbol pin, nor hand-written JSON.
//!
//! Claims the **Wire Harness** panel (the connection list — its own) AND the
//! two assembly panels, so a harness document has its components and their
//! constraints to hand. Routing itself is automatic — every history run
//! re-routes the connections at its tail — so there is no Route button.

use super::{ButtonState, FeatureInfo, Workbench, WorkbenchButton};

/// The Wire Harness connection panel's id (claim + registration key).
pub const PANEL_ID: &str = "wireHarness";

/// Declare this part's FIRST connection point, which is what opens the Qualify
/// pane. Offered only while the part declares none — once it does, the pane is
/// on screen and every further group and point is added there.
pub const DECLARE_POINT_BUTTON_ID: &str = "wireharness.declare_point";

/// The `ports` block a press writes: one group, one point at the part origin.
/// `wiring` because this is the harness workbench — a pin-minted group is
/// `pcb`, and the purpose is a dropdown in the pane either way.
pub const FIRST_GROUP: &str = "J1";
pub const FIRST_POINT: &str = "1";
pub const FIRST_PURPOSE: &str = "wiring";

/// Datum / Plane / Sketch / Spline / Waypoint / Component.
fn includes(feature: &FeatureInfo<'_>) -> bool {
    matches!(feature.type_code, "S" | "D" | "P" | "SP" | "WP" | "ACOMP")
}

/// Whether the part declares NO connection point yet — the one condition the
/// button below carries, and the exact inverse of the Qualify pane's.
fn declares_none(state: &ButtonState) -> bool {
    !super::declares_connection_points(state)
}

/// One button: the way in to a part's connection points.
static BUTTONS: &[WorkbenchButton] = &[WorkbenchButton {
    id: DECLARE_POINT_BUTTON_ID,
    // U+22C8 ⋈ — a socket, a bar and a pin row: a connector seen end on.
    glyph: "\u{22C8}",
    tooltip: "Declare connection point",
    when: Some(declares_none),
    pressed: None,
    disabled: None,
    caption: None,
    menu: &[],
}];

/// The connection panel plus the two assembly panels.
static PANELS: &[&str] = &[
    PANEL_ID,
    super::assembly::CONSTRAINTS_PANEL_ID,
    super::assembly::BOM_PANEL_ID,
];

pub static WIRE_HARNESS: Workbench = Workbench {
    id: "wireHarness",
    label: "Wire harness",
    glyph: "\u{E063}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
