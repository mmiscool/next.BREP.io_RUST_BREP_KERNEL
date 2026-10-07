//! The hit-key registry: each panel documents the key prefixes it publishes
//! in its hit-rect blob, beside the publisher. `brep://widgets` is generated
//! from [`hit_key_docs`], and a `test-mcp` script checks that every key the
//! running app publishes matches a documented prefix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HitKeyDoc {
    /// The panel name as `Registry::hit_blob_panel` derives it.
    pub panel: &'static str,
    /// A key prefix (`step:`), or a whole key (`form:return`).
    pub prefix: &'static str,
    pub meaning: &'static str,
    /// The APP COMMAND that does what a click here does, when one exists
    /// ([`crate::automation::command::registry`]).
    ///
    /// A host may expose it under another tool name — the MCP server owns the
    /// file halves, so `doc_load` reaches a caller as `document_open` — and the
    /// generated widget docs translate it. This side names the command, because
    /// this side is the only one that can check the name is real.
    ///
    /// This is what makes "every button is reachable without a pointer" a
    /// CHECKED property rather than a claim: the registration sits beside the
    /// widget it describes, the generated widget docs carry it, and
    /// `every_named_command_exists` fails on a name the registry does not have.
    /// `None` is the honest answer for a widget with no command equivalent (a
    /// history row, a form field) — those are driven by clicking their rect.
    pub command: Option<&'static str>,
}

/// Every panel's registrations. Adding a panel means adding its static here —
/// and the `brep-mcp test` hit-key script fails on any published key that no
/// registration covers, so a forgotten line is caught by the gate.
pub fn hit_key_docs() -> Vec<&'static HitKeyDoc> {
    let sets: &[&[HitKeyDoc]] = &[
        crate::plugins::HIT_KEYS,
        crate::panels::history::HIT_KEYS,
        crate::palette::HIT_KEYS,
        crate::panels::toolbar::HIT_KEYS,
        crate::panels::workbench_toolbar::HIT_KEYS,
        crate::panels::file::HIT_KEYS,
        crate::panels::file_explorer::HIT_KEYS,
        crate::panels::plm_host::HIT_KEYS,
        crate::panels::scene::HIT_KEYS,
        crate::panels::selection::HIT_KEYS,
        crate::panels::part_properties::HIT_KEYS,
        crate::panels::context_bar::HIT_KEYS,
        crate::panels::mode_bar::HIT_KEYS,
        crate::panels::expressions::HIT_KEYS,
        crate::panels::document_tabs::HIT_KEYS,
        crate::panels::settings::HIT_KEYS,
        crate::panels::info::HIT_KEYS,
        crate::panels::info_windows::HIT_KEYS,
        crate::panels::interference::HIT_KEYS,
        crate::panels::auto_constraints::HIT_KEYS,
        crate::panels::step_parts::HIT_KEYS,
        crate::panels::wire_harness::HIT_KEYS,
        crate::panels::bug_report::HIT_KEYS,
        crate::javascript::HIT_KEYS,
        crate::panels::pmi::HIT_KEYS,
        crate::panels::qualify::HIT_KEYS,
        crate::panels::family_table_editor::HIT_KEYS,
        crate::panels::sheets::HIT_KEYS,
        crate::panels::bom::HIT_KEYS,
        crate::panels::ecad_parts::HIT_KEYS,
        crate::panels::assembly_constraints::HIT_KEYS,
        crate::panels::sketch::HIT_KEYS,
        crate::panels::spline_anchors::HIT_KEYS,
        crate::panels::stl_import::HIT_KEYS,
        crate::recovery::HIT_KEYS,
        crate::panels::toasts::HIT_KEYS,
        crate::panels::busy::HIT_KEYS,
        crate::viewport::HIT_KEYS,
    ];
    sets.iter().flat_map(|s| s.iter()).collect()
}

/// Does `panel/key` match a registered prefix? A prefix ending in `:` names a
/// family (`sheet/anchor:<ref>`); any other names one key, or an INDEXED run of
/// it — `refsel:x` documents `refsel:x0`, `refsel:x1`, … and nothing else.
pub fn documented(panel: &str, key: &str) -> bool {
    hit_key_docs().iter().any(|d| {
        d.panel == panel
            && (key == d.prefix
                || (d.prefix.ends_with(':') && key.starts_with(d.prefix))
                || key
                    .strip_prefix(d.prefix)
                    .is_some_and(|index| index.bytes().all(|b| b.is_ascii_digit())))
    })
}

/// The docs as JSON: `[{panel, prefix, meaning, command}]`.
pub fn hit_key_docs_json() -> serde_json::Value {
    serde_json::Value::Array(
        hit_key_docs()
            .iter()
            .map(|d| serde_json::json!({ "panel": d.panel, "prefix": d.prefix, "meaning": d.meaning, "command": d.command }))
            .collect(),
    )
}

