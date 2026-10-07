//! The SKETCH mode's draw tools, as workbench buttons EVERY workbench carries.
//!
//! Sketch mode is a shell TAKEOVER, not one workbench's mode: it is entered
//! from a sketch feature's form (`editSketch` → [`EngineState::enter_sketch_mode`]),
//! the History panel is unclaimed and therefore visible under every workbench,
//! and a workbench never changes what the history lets you edit (see the
//! [module doc](super)). So no workbench OWNS these tools — declaring them on
//! Modeling would take them away the moment you sketched under Sheet Metal or
//! Assembly, which is exactly the incoherence this slice removes.
//!
//! They are therefore SHARED buttons: declared once here and appended to every
//! workbench's row by [`super::shared_buttons`], conditional
//! ([`WorkbenchButton::when`]) on a sketch actually being edited. Out of sketch
//! mode the row is unchanged; in it, every workbench grows the same eleven
//! buttons, and switching workbench mid-sketch keeps them.
//!
//! This replaced the sketch-mode DRAW-TOOLS STRIP (`panels::sketch`'s
//! `show_mode_bar`, a `Panel::top` of its own): the last of the three homes for
//! "this mode's tools". The mode-EXIT card (Finish / Cancel, `panels::mode_bar`)
//! stays where it is — it is the exit, not a tool, and the user's "one
//! predictable corner" direction puts every mode's exit there.
//!
//! [`EngineState::enter_sketch_mode`]: brep_render::engine_state::EngineState::enter_sketch_mode

use super::{ButtonState, WorkbenchButton};

/// The one-shot AUTO-CONSTRAIN action's button id — not a tool and not a
/// toggle: it infers the constraints the rough-in geometry already implies and
/// is done.
pub const AUTOCONSTRAIN_BUTTON_ID: &str = "sketch.autoconstrain";

/// The draw tools, as `(button id, engine tool id)` — ONE table, read by the
/// pressed predicate below and by `BrepApp::dispatch_workbench_button`. The
/// ENGINE TOOL ID is what [`EngineState::sketch_set_tool`] takes and what
/// `__brepSketch.tool` reads back, so it is the half that must not drift; the
/// button id is the automation surface's (`workbench:btn:sketch.tool.line`, or
/// the `workbench_button` command's `id`). The glyph and the tooltip live in
/// the declaration alone, the way PMI's `DIM_TOOLS` keeps only the kind and the
/// alignment.
///
/// `select` is the neutral state rather than a tool — the engine normalizes it
/// to `None` — which is why it is in the table anyway: it is how you put a tool
/// DOWN, and a row that offers every tool but no way back to picking would be a
/// trap.
///
/// [`EngineState::sketch_set_tool`]: brep_render::engine_state::EngineState::sketch_set_tool
pub const DRAW_TOOLS: &[(&str, &str)] = &[
    ("sketch.tool.select", "select"),
    ("sketch.tool.point", "point"),
    ("sketch.tool.line", "line"),
    ("sketch.tool.rect", "rect"),
    ("sketch.tool.circle", "circle"),
    ("sketch.tool.arc", "arc"),
    ("sketch.tool.bezier", "bezier"),
    ("sketch.tool.handdraw", "handdraw"),
    ("sketch.tool.trim", "trim"),
    ("sketch.tool.pickEdges", "pickEdges"),
];

/// The ENGINE tool id a draw-tool button arms, or `None` for any other id —
/// the dispatch side's reading of [`DRAW_TOOLS`].
pub fn draw_tool(id: &str) -> Option<&'static str> {
    DRAW_TOOLS.iter().find(|(button, _)| *button == id).map(|(_, tool)| *tool)
}

/// A sketch is being edited — the condition every one of these buttons is
/// offered under. Out of sketch mode there is nothing to draw ON, so they are
/// simply not in the row.
fn sketch_is_open(state: &ButtonState) -> bool {
    state.engine.sketch_mode()
}

/// Whether the armed tool IS the one `id` names — the pressed predicate,
/// shared by all ten so the table stays the only list. `select` is the armed
/// tool exactly when nothing else is (the engine stores it as `None`).
fn tool_armed(state: &ButtonState, id: &str) -> bool {
    let Some(tool) = draw_tool(id) else {
        return false;
    };
    state.engine.sketch_active_tool().unwrap_or("select") == tool
}

/// The eleven buttons, in the strip's old order: the ten tools, then the
/// one-shot Auto-constrain.
///
/// Every glyph is catalogued artwork (this app ships no font fallback, so an
/// uncatalogued character paints as tofu). Nine of the ten tools kept the
/// picture the strip drew; the LINE tool's was a bare ASCII `/`, which has no
/// artwork and would have been the one tofu in the row, so it gained
/// **U+E06E** — the next free private-use slot after the sheet tools' E068-E06D
/// and clear of the retired E05A-E05C block. E06E is outside `icons`'
/// `FEATURE_BLOCKS` and `WORKBENCHES` ranges, so monochrome ink is right for it
/// by construction.
pub static BUTTONS: &[WorkbenchButton] = &[
    WorkbenchButton {
        id: "sketch.tool.select",
        glyph: "\u{1F446}",
        ribbon_path: "Home/Sketch/Select   drag entities", size: crate::workbench::CommandSize::Compact, tooltip: "Select / drag entities",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.select")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.point",
        glyph: "\u{2316}",
        ribbon_path: "Home/Sketch/Place a point", size: crate::workbench::CommandSize::Compact, tooltip: "Place a point",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.point")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.line",
        glyph: "\u{E06E}",
        ribbon_path: "Home/Sketch/Draw connected line segments", size: crate::workbench::CommandSize::Compact, tooltip: "Draw connected line segments (Esc ends)",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.line")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.rect",
        glyph: "\u{2610}",
        ribbon_path: "Home/Sketch/Draw a rectangle", size: crate::workbench::CommandSize::Compact, tooltip: "Draw a rectangle (two opposite corners)",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.rect")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.circle",
        glyph: "\u{25EF}",
        ribbon_path: "Home/Sketch/Draw a circle", size: crate::workbench::CommandSize::Compact, tooltip: "Draw a circle (center, then radius)",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.circle")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.arc",
        glyph: "\u{25E0}",
        ribbon_path: "Home/Sketch/Draw an arc", size: crate::workbench::CommandSize::Compact, tooltip: "Draw an arc (center, start, end)",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.arc")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.bezier",
        glyph: "\u{223F}",
        ribbon_path: "Home/Sketch/Bezier", size: crate::workbench::CommandSize::Compact, tooltip: "Bezier \u{2014} end, ctrl, ctrl, end; click a spline to add a point",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.bezier")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.handdraw",
        glyph: "\u{270D}",
        ribbon_path: "Home/Sketch/Freehand", size: crate::workbench::CommandSize::Compact, tooltip: "Freehand (auto line/circle/arc)",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.handdraw")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.trim",
        glyph: "\u{2702}",
        ribbon_path: "Home/Sketch/Trim curve", size: crate::workbench::CommandSize::Compact, tooltip: "Trim curve",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.trim")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "sketch.tool.pickEdges",
        glyph: "\u{26D3}",
        ribbon_path: "Home/Sketch/Link external edge", size: crate::workbench::CommandSize::Compact, tooltip: "Link external edge",
        when: Some(sketch_is_open),
        pressed: Some(|state| tool_armed(state, "sketch.tool.pickEdges")),
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: AUTOCONSTRAIN_BUTTON_ID,
        // 🤖 (U+1F916) — "auto / do it for me", the glyph the strip used.
        glyph: "\u{1F916}",
        ribbon_path: "Home/Sketch/Auto-constrain: infer coincident + horizontal vertical from the geometry", size: crate::workbench::CommandSize::Compact, tooltip: "Auto-constrain: infer coincident + horizontal/vertical from the geometry",
        when: Some(sketch_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
];
