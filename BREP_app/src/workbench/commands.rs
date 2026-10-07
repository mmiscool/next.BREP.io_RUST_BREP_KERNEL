//! Renderer-neutral command snapshots. Availability and dispatch belong to their owners.
use super::{ButtonState, WorkbenchButton};
use brep_render::engine_state::EngineState;

pub const RIBBON_TABS: [&str; 3] = ["Home", "View", "Help"];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandSize {
    Large,
    Compact,
}

/// Built-ins borrow static strings; runtime adapters borrow their owned metadata.
#[derive(Clone, Copy, Debug)]
pub struct CommandPresentation<'a> {
    pub id: &'a str,
    pub glyph: &'a str,
    pub ribbon_path: &'a str,
    pub size: CommandSize,
}
impl<'a> CommandPresentation<'a> {
    pub fn label(&self) -> &'a str {
        self.ribbon_path.rsplit('/').next().unwrap_or("")
    }
}

pub fn ribbon_segments(path: &str) -> Result<[&str; 3], String> {
    let parts: Vec<_> = path.split('/').collect();
    if parts.len() != 3 || parts.iter().any(|p| p.trim().is_empty() || p.trim() != *p) {
        return Err(format!("invalid ribbon path: {path:?}"));
    }
    if !RIBBON_TABS.contains(&parts[0]) {
        return Err(format!("unknown ribbon tab: {}", parts[0]));
    }
    Ok([parts[0], parts[1], parts[2]])
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CommandTarget {
    Shell(&'static str),
    Workbench(&'static str),
    Plugin(String),
    Feature(String),
    Constraint(String),
    Annotation(String),
}

#[derive(Clone, Debug)]
pub struct OfferedCommand {
    pub id: String,
    pub glyph: String,
    pub ribbon_path: String,
    pub size: CommandSize,
    pub target: CommandTarget,
    pub pressed: bool,
    pub toggle: bool,
    pub disabled_reason: Option<String>,
    pub detail: String,
    pub menu: Vec<OfferedCommand>,
}
impl OfferedCommand {
    pub fn presentation(&self) -> CommandPresentation<'_> {
        CommandPresentation {
            id: &self.id,
            glyph: &self.glyph,
            ribbon_path: &self.ribbon_path,
            size: self.size,
        }
    }
    pub fn segments(&self) -> [&str; 3] {
        ribbon_segments(self.presentation().ribbon_path).expect("validated command path")
    }
    pub fn label(&self) -> &str {
        self.presentation().label()
    }
    pub fn is_home(&self) -> bool {
        self.segments()[0] == "Home"
    }
    /// A creation command: Classic draws these on its creation strip, and
    /// everything else (File, Undo, actions, tools) on its primary row.
    pub fn is_creation(&self) -> bool {
        matches!(
            self.target,
            CommandTarget::Feature(_) | CommandTarget::Constraint(_) | CommandTarget::Annotation(_)
        )
    }
}

pub fn validate_commands(commands: &[OfferedCommand]) -> Result<(), String> {
    fn visit(
        commands: &[OfferedCommand],
        ids: &mut std::collections::HashSet<String>,
    ) -> Result<(), String> {
        for command in commands {
            ribbon_segments(&command.ribbon_path)?;
            if command.id.is_empty() || !ids.insert(command.id.clone()) {
                return Err(format!("duplicate or empty command id: {}", command.id));
            }
            visit(&command.menu, ids)?;
        }
        Ok(())
    }
    visit(commands, &mut Default::default())
}

fn static_command(p: CommandPresentation<'static>, target: CommandTarget) -> OfferedCommand {
    OfferedCommand {
        id: p.id.into(),
        glyph: p.glyph.into(),
        ribbon_path: p.ribbon_path.into(),
        size: p.size,
        target,
        pressed: false,
        toggle: false,
        disabled_reason: None,
        detail: p.label().into(),
        menu: vec![],
    }
}

/// File commands share metadata; Properties and Settings are offered only in File.
pub const FILE_COMMANDS: &[CommandPresentation<'static>] = &[
    CommandPresentation {
        id: "file:new",
        glyph: "\u{E010}",
        ribbon_path: "Home/File/New",
        size: CommandSize::Large,
    },
    CommandPresentation {
        id: "file:open",
        glyph: "\u{E011}",
        ribbon_path: "Home/File/Open",
        size: CommandSize::Large,
    },
    CommandPresentation {
        id: "file:save",
        glyph: "\u{E012}",
        ribbon_path: "Home/File/Save",
        size: CommandSize::Large,
    },
    CommandPresentation {
        id: "file:saveas",
        glyph: "\u{E013}",
        ribbon_path: "Home/File/Save As",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "file:import",
        glyph: "\u{E014}",
        ribbon_path: "Home/File/Import",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "file:export",
        glyph: "\u{E015}",
        ribbon_path: "Home/File/Export",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "properties",
        glyph: "\u{E066}",
        ribbon_path: "Home/File/Document Properties",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "settings",
        glyph: "\u{2699}",
        ribbon_path: "Home/File/Settings",
        size: CommandSize::Compact,
    },
    // Plugin management lives under Settings in the File menu, in both styles.
    CommandPresentation {
        id: "workbench:manage:plugins",
        glyph: "\u{2699}",
        ribbon_path: "Home/File/Plugins",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "workbench:manage:javascript",
        glyph: "\u{270E}",
        ribbon_path: "Home/File/JavaScript",
        size: CommandSize::Compact,
    },
];

/// Flat New rows in DocumentClass::ALL order.
pub const NEW_DOCUMENT_COMMANDS: &[CommandPresentation<'static>] = &[
    CommandPresentation {
        id: "file:newclass:normal",
        glyph: "\u{E010}",
        ribbon_path: "Home/File/New part",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "file:newclass:family",
        glyph: "\u{E010}",
        ribbon_path: "Home/File/New family",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "file:newclass:template",
        glyph: "\u{E010}",
        ribbon_path: "Home/File/New template",
        size: CommandSize::Compact,
    },
];

pub fn file_presentation(id: &str) -> CommandPresentation<'static> {
    *FILE_COMMANDS
        .iter()
        .find(|c| c.id == id)
        .expect("File command is declared")
}

pub const SHELL_COMMANDS: &[CommandPresentation<'static>] = &[
    CommandPresentation {
        id: "undo",
        glyph: "\u{E016}",
        ribbon_path: "Home/Undo/Undo",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "redo",
        glyph: "\u{E017}",
        ribbon_path: "Home/Undo/Redo",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "wireframe",
        glyph: "\u{1F578}",
        ribbon_path: "View/Display/Wireframe",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "projection",
        glyph: "\u{E018}",
        ribbon_path: "View/Display/Projection",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "show:faces",
        glyph: "\u{E028}",
        ribbon_path: "View/Display/Show faces",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "show:edges",
        glyph: "\u{E029}",
        ribbon_path: "View/Display/Show edges",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "show:vertices",
        glyph: "\u{E02A}",
        ribbon_path: "View/Display/Show vertices",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "fit",
        glyph: "\u{26F6}",
        ribbon_path: "View/Navigation/Zoom to fit",
        size: CommandSize::Large,
    },
    CommandPresentation {
        id: "help",
        glyph: "\u{2753}",
        ribbon_path: "Help/Support/Help",
        size: CommandSize::Large,
    },
    CommandPresentation {
        id: "info",
        glyph: "\u{2139}",
        ribbon_path: "Help/Support/Info",
        size: CommandSize::Compact,
    },
    CommandPresentation {
        id: "bug",
        glyph: "\u{1F41E}",
        ribbon_path: "Help/Support/Submit Bug",
        size: CommandSize::Compact,
    },
];

pub(crate) fn adapt_button(button: &WorkbenchButton, state: &ButtonState) -> OfferedCommand {
    let mut c = static_command(button.presentation(), CommandTarget::Workbench(button.id));
    c.pressed = button.is_pressed(state);
    c.toggle = button.pressed.is_some();
    c.disabled_reason = button.disabled_reason(state).map(str::to_owned);
    c.detail = button.detail(state);
    c.menu = button
        .menu
        .iter()
        .filter(|b| b.offered(state))
        .map(|b| adapt_button(b, state))
        .collect();
    c
}

/// Canonical feature command text for text-and-icon surfaces such as palettes.
pub fn feature_command_label(feature: &serde_json::Value) -> String {
    let ty = feature["type"].as_str().unwrap_or("");
    let label = feature["ribbonPath"]
        .as_str()
        .and_then(|p| p.rsplit('/').next())
        .or(feature["longName"].as_str())
        .or(feature["label"].as_str())
        .unwrap_or(ty);
    let glyph = brep_render::features::feature_icon(ty)
        .map(|g| g.to_string())
        .or_else(|| feature["icon"].as_str().map(str::to_owned));
    glyph.map_or_else(|| label.to_owned(), |g| format!("{g} {label}"))
}

/// The same Home descriptors feed the Classic strip and Ribbon. Order is catalogue order.
pub fn offered_home_commands(engine: &EngineState) -> Vec<OfferedCommand> {
    if !engine.settings.show_workbench_toolbar || engine.sketch_mode() || engine.ref_select_active()
    {
        return vec![];
    }
    let active = &engine.settings.workbench;
    let dynamic = super::owned_workbenches(engine);
    let active_owned = dynamic.iter().find(|w| w.id == *active);
    let mut commands = vec![];
    let catalogue = engine.feature_catalogue();
    for f in catalogue["features"].as_array().into_iter().flatten() {
        let Some(ty) = f["type"].as_str() else {
            continue;
        };
        if !active_owned.map_or_else(
            || super::includes_feature(active, ty),
            |w| w.feature_types.iter().any(|f| f == ty),
        ) {
            continue;
        }
        // Older runtime plugins are adapted at this boundary; built-ins declare paths in their schemas.
        let label = f["label"].as_str().or(f["longName"].as_str()).unwrap_or(ty);
        let path = f["ribbonPath"]
            .as_str()
            .map(str::to_owned)
            .unwrap_or_else(|| format!("Home/Extensions/{}", label.replace('/', " ")));
        commands.push(OfferedCommand {
            id: format!("feature:{ty}"),
            glyph: brep_render::features::feature_icon(ty)
                .map(|glyph| glyph.to_string())
                .or_else(|| f["icon"].as_str().map(str::to_owned))
                .unwrap_or_else(|| ty.into()),
            ribbon_path: path,
            size: if f["commandSize"] == "Compact" {
                CommandSize::Compact
            } else {
                CommandSize::Large
            },
            target: CommandTarget::Feature(ty.into()),
            pressed: false,
            toggle: false,
            disabled_reason: None,
            detail: format!("Add {label}"),
            menu: vec![],
        });
    }
    let state = ButtonState::of(engine);
    if super::panel_visible(active, super::assembly::CONSTRAINTS_PANEL_ID, &state) {
        for d in brep_render::brep_kernel::CONSTRAINT_TYPES {
            commands.push(creation(
                format!("constraint:{}", d.type_id),
                d.icon,
                format!("Home/Constraints/{}", d.label),
                CommandTarget::Constraint(d.type_id.into()),
            ));
        }
    }
    if super::panel_visible(active, super::pmi::PANEL_ID, &state) {
        for d in brep_render::brep_kernel::PMI_TYPES {
            commands.push(creation(
                format!("annotation:{}", d.type_id),
                d.icon,
                format!("Home/Annotations/{}", d.label),
                CommandTarget::Annotation(d.type_id.into()),
            ));
        }
    }
    commands
}
fn creation(id: String, glyph: &str, path: String, target: CommandTarget) -> OfferedCommand {
    let label = path.rsplit('/').next().unwrap_or("");
    let detail = match &target {
        CommandTarget::Constraint(_) => format!("Add {label} constraint from the selection"),
        CommandTarget::Annotation(_) => format!(
            "Add a {} to the active view, from the selection",
            label.to_lowercase()
        ),
        _ => String::new(),
    };
    OfferedCommand {
        id,
        glyph: glyph.into(),
        ribbon_path: path,
        size: CommandSize::Compact,
        target,
        pressed: false,
        toggle: false,
        disabled_reason: None,
        detail,
        menu: vec![],
    }
}

pub fn offered_commands(state: &ButtonState, info_open: bool) -> Vec<OfferedCommand> {
    #[cfg(debug_assertions)]
    {
        static VALIDATED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
        VALIDATED.get_or_init(|| {
            validate_builtin_presentations().expect("invalid built-in command registry")
        });
    }
    let engine = state.engine;
    let mut commands: Vec<_> = FILE_COMMANDS
        .iter()
        .filter(|p| p.id.starts_with("file:"))
        .chain(SHELL_COMMANDS)
        .map(|p| {
            let mut c = static_command(*p, CommandTarget::Shell(p.id));
            c.toggle = matches!(
                p.id,
                "wireframe" | "projection" | "show:faces" | "show:edges" | "show:vertices" | "info"
            );
            c.pressed = match p.id {
                "wireframe" => engine.settings.wireframe,
                "projection" => matches!(
                    engine.camera.projection,
                    brep_render::view::Projection::Perspective { .. }
                ),
                "show:faces" => engine.settings.show_faces,
                "show:edges" => engine.settings.show_edges,
                "show:vertices" => engine.settings.show_vertices,
                "info" => info_open,
                _ => false,
            };
            let unavailable = match p.id {
                "undo" => {
                    !(if engine.sketch_mode() {
                        engine.sketch_can_undo()
                    } else {
                        engine.can_undo()
                    })
                }
                "redo" => {
                    !(if engine.sketch_mode() {
                        engine.sketch_can_redo()
                    } else {
                        engine.can_redo()
                    })
                }
                _ => false,
            };
            if unavailable {
                c.disabled_reason = Some("No history step available".into());
            }
            match p.id {
                "undo" if engine.sketch_mode() => c.detail = "Undo sketch edit (Ctrl+Z)".into(),
                "redo" if engine.sketch_mode() => c.detail = "Redo sketch edit (Ctrl+Y)".into(),
                "fit" if engine.sheet_open().is_some() => {
                    c.detail = "Zoom to fit (the open sheet's paper)".into()
                }
                "info" => c.detail = "Info (licences and diagnostics)".into(),
                _ => {}
            }
            if p.id == "projection" {
                c.detail = if c.pressed {
                    "Perspective projection"
                } else {
                    "Orthographic projection"
                }
                .into();
            }
            c
        })
        .collect();
    commands.extend(
        super::offered_buttons(&engine.settings.workbench, state)
            .into_iter()
            .map(|b| adapt_button(b, state)),
    );
    for a in super::plugin_actions(engine, &engine.settings.workbench) {
        let Some(id) = a["id"].as_str() else { continue };
        let label = a["label"].as_str().unwrap_or(id);
        commands.push(OfferedCommand {
            id: id.into(),
            glyph: a["glyph"].as_str().unwrap_or("\u{2699}").into(),
            ribbon_path: a["ribbonPath"]
                .as_str()
                .map(str::to_owned)
                .unwrap_or_else(|| format!("Home/Extensions/{}", label.replace('/', " "))),
            size: if a["commandSize"] == "Large" {
                CommandSize::Large
            } else {
                CommandSize::Compact
            },
            target: CommandTarget::Plugin(id.into()),
            pressed: false,
            toggle: false,
            disabled_reason: None,
            detail: label.into(),
            menu: vec![],
        });
    }
    commands.extend(offered_home_commands(engine));
    debug_assert!(
        validate_commands(&commands).is_ok(),
        "invalid command registry: {:?}",
        validate_commands(&commands)
    );
    commands
}

/// Groups are derived in first-declaration order; commands retain source order within them.
pub fn command_groups<'a>(
    commands: &'a [OfferedCommand],
    tab: &str,
) -> Vec<(&'a str, Vec<&'a OfferedCommand>)> {
    let mut groups: Vec<(&str, Vec<&OfferedCommand>)> = vec![];
    for c in commands.iter().filter(|c| c.segments()[0] == tab) {
        let group = c.segments()[1];
        if let Some((_, members)) = groups.iter_mut().find(|(name, _)| *name == group) {
            members.push(c);
        } else {
            groups.push((group, vec![c]));
        }
    }
    groups
}

/// Validate all declarations, including commands not currently offered.
pub fn validate_builtin_presentations() -> Result<(), String> {
    let mut seen = std::collections::HashSet::new();
    let catalogue = brep_render::features::feature_catalogue();
    for p in FILE_COMMANDS
        .iter()
        .chain(NEW_DOCUMENT_COMMANDS)
        .chain(SHELL_COMMANDS)
        .copied()
        .chain(super::every_button_in(super::WORKBENCHES).map(|b| b.presentation()))
    {
        ribbon_segments(p.ribbon_path)?;
        if !seen.insert(p.id.to_owned()) {
            return Err(format!("duplicate command id: {}", p.id));
        }
    }
    for feature in catalogue["features"].as_array().into_iter().flatten() {
        let path = feature["ribbonPath"]
            .as_str()
            .ok_or_else(|| format!("feature has no ribbonPath: {}", feature["type"]))?;
        if ribbon_segments(path)?[0] != "Home" {
            return Err(format!("feature outside Home: {path}"));
        }
        if !matches!(feature["commandSize"].as_str(), Some("Large" | "Compact")) {
            return Err(format!("feature has no commandSize: {path}"));
        }
        let ty = feature["type"].as_str().ok_or("feature has no type")?;
        if !seen.insert(format!("feature:{ty}")) {
            return Err(format!("duplicate feature type: {ty}"));
        }
    }
    Ok(())
}

