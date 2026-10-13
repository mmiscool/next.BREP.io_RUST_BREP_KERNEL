//! The 3D SKETCH mode's tools, as workbench buttons EVERY workbench carries —
//! the 3D sibling of [`super::sketch`]. Like the planar tools they are SHARED
//! buttons: declared once here, appended to every workbench's row by
//! [`super::shared_buttons`], and offered only while a 3D sketch is being
//! edited ([`EngineState::sketch3d_mode`]), so out of that mode the rows are
//! unchanged and in it every workbench grows the same nine buttons.
//!
//! Two kinds sit here. The DRAW TOOLS (`sketch3d.tool.*`) arm one tool each —
//! [`EngineState::sketch3d_set_tool`] takes the engine tool id — and the lit
//! one is the armed one. The WORKING PLANE buttons (`sketch3d.plane.*`) are a
//! radio of three: a click with no model geometry under the cursor lands on
//! the working plane, one of the three world planes through the last placed
//! point, and the lit button is the plane in force
//! ([`EngineState::sketch3d_set_work_plane`]; Tab cycles them).
//!
//! [`EngineState::sketch3d_mode`]: brep_render::engine_state::EngineState::sketch3d_mode
//! [`EngineState::sketch3d_set_tool`]: brep_render::engine_state::EngineState::sketch3d_set_tool
//! [`EngineState::sketch3d_set_work_plane`]: brep_render::engine_state::EngineState::sketch3d_set_work_plane

use super::{ButtonState, WorkbenchButton};

/// The draw tools, as `(button id, engine tool id)` — one table, read by the
/// pressed predicate and by `BrepApp::dispatch_workbench_button`.
pub const DRAW_TOOLS: &[(&str, &str)] = &[
    ("sketch3d.tool.select", "select"),
    ("sketch3d.tool.point", "point"),
    ("sketch3d.tool.line", "line"),
    ("sketch3d.tool.arc", "arc"),
    ("sketch3d.tool.bezier", "bezier"),
    ("sketch3d.tool.pickRef", "pickRef"),
];

/// The working planes, as `(button id, plane name)`.
pub const WORK_PLANES: &[(&str, &str)] = &[
    ("sketch3d.plane.xy", "XY"),
    ("sketch3d.plane.yz", "YZ"),
    ("sketch3d.plane.xz", "XZ"),
];

/// The ENGINE tool id a draw-tool button arms, or `None` for any other id.
pub fn draw_tool(id: &str) -> Option<&'static str> {
    DRAW_TOOLS.iter().find(|(button, _)| *button == id).map(|(_, tool)| *tool)
}

/// The working plane a plane button selects, or `None` for any other id.
pub fn work_plane(id: &str) -> Option<&'static str> {
    WORK_PLANES.iter().find(|(button, _)| *button == id).map(|(_, plane)| *plane)
}

/// A 3D sketch is being edited — the condition every one of these buttons is
/// offered under.
fn sketch3d_is_open(state: &ButtonState) -> bool {
    state.engine.sketch3d_mode()
}

fn tool_armed(state: &ButtonState, id: &str) -> bool {
    let Some(tool) = draw_tool(id) else {
        return false;
    };
    state.engine.sketch3d_active_tool().unwrap_or("select") == tool
}

fn plane_in_force(state: &ButtonState, id: &str) -> bool {
    let Some(plane) = work_plane(id) else {
        return false;
    };
    state.engine.sketch3d_work_plane() == Some(plane)
}

/// The nine buttons: the six tools, then the three working planes.
///
/// The tool glyphs are the planar tools' catalogued artwork where the tool is
/// the same gesture (Select, Point, Line, Arc, Bezier, Link); the working
/// planes have three monochrome icons of their own, **U+E0C3**-**U+E0C5**,
/// the next free private-use slots after the eCAD sheet symbols' E0C0-E0C2.
pub static BUTTONS: &[WorkbenchButton] = &[
    WorkbenchButton {
        id: "sketch3d.tool.select",
        glyph: "\u{1F446}",
        ribbon_path: "Home/3D Sketch/Select   drag points", size: crate::workbench::CommandSize::Compact, tooltip: "Select / drag points and curves",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| tool_armed(state, "sketch3d.tool.select")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.tool.point",
        glyph: "\u{2316}",
        ribbon_path: "Home/3D Sketch/Place a point", size: crate::workbench::CommandSize::Compact, tooltip: "Place a point (on model geometry under the cursor, else on the working plane)",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| tool_armed(state, "sketch3d.tool.point")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.tool.line",
        glyph: "\u{E06E}",
        ribbon_path: "Home/3D Sketch/Draw connected line segments", size: crate::workbench::CommandSize::Compact, tooltip: "Draw connected line segments in 3D (Esc ends the chain; Tab switches the working plane)",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| tool_armed(state, "sketch3d.tool.line")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.tool.arc",
        glyph: "\u{25E0}",
        ribbon_path: "Home/3D Sketch/Draw an arc", size: crate::workbench::CommandSize::Compact, tooltip: "Draw an arc (center, start, end)",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| tool_armed(state, "sketch3d.tool.arc")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.tool.bezier",
        glyph: "\u{223F}",
        ribbon_path: "Home/3D Sketch/Bezier", size: crate::workbench::CommandSize::Compact, tooltip: "Bezier \u{2014} end, ctrl, ctrl, end",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| tool_armed(state, "sketch3d.tool.bezier")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.tool.pickRef",
        glyph: "\u{26D3}",
        ribbon_path: "Home/3D Sketch/Link model reference", size: crate::workbench::CommandSize::Compact, tooltip: "Link a model vertex, edge, face or datum plane into the sketch as a fixed reference",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| tool_armed(state, "sketch3d.tool.pickRef")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.plane.xy",
        glyph: "\u{E0C3}",
        ribbon_path: "Home/3D Sketch/Working plane XY", size: crate::workbench::CommandSize::Compact, tooltip: "Working plane XY: a free click lands on the XY plane through the last placed point",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| plane_in_force(state, "sketch3d.plane.xy")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.plane.yz",
        glyph: "\u{E0C4}",
        ribbon_path: "Home/3D Sketch/Working plane YZ", size: crate::workbench::CommandSize::Compact, tooltip: "Working plane YZ: a free click lands on the YZ plane through the last placed point",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| plane_in_force(state, "sketch3d.plane.yz")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch3d.plane.xz",
        glyph: "\u{E0C5}",
        ribbon_path: "Home/3D Sketch/Working plane XZ", size: crate::workbench::CommandSize::Compact, tooltip: "Working plane XZ: a free click lands on the XZ plane through the last placed point",
        when: Some(sketch3d_is_open),
        pressed: Some(|state| plane_in_force(state, "sketch3d.plane.xz")),
        disabled: None,
        caption: None,
        menu: &[],
    },
];
