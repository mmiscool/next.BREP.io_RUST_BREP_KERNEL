//! The **Info** window — the licences, and what this build is running on.
//!
//! The toolbar's ℹ button opens it. Two things live here because a user asking
//! either question is in the same place: "what am I allowed to do with this"
//! and "what is it running on".
//!
//! * **Licences.** The project licence and the maintained third-party notices,
//!   compiled into the binary from the very files the help site renders
//!   (`LICENSE.md`, `THIRD-PARTY-NOTICES.md`) — so a build that is not served
//!   beside its `web/help/` still carries them. The GENERATED crate inventory
//!   (every shipped dependency, grouped by licence expression) is built from
//!   `cargo metadata` at docs time and cannot be compiled in, so the window
//!   links to its help page instead.
//! * **Diagnostics.** [`crate::diagnostics::Diagnostics`], rendered row for row.
//!   The window OWNS no diagnostic of its own: it is handed the app's one
//!   instance, the same one the problem report embeds, so what a user reads
//!   here and what a triager reads in their report cannot differ.
//! * **Connect an agent (MCP).** A button that starts the embedded MCP server
//!   at runtime — what `brep-app --mcp` does at launch — and, once it runs,
//!   the very text the console prints for pointing an agent at this window,
//!   selectable and with a Copy button. It is here, beside the diagnostics,
//!   because the URL and the session root are facts about THIS session, like
//!   the renderer: not a setting that persists. The window owns none of the
//!   state: it is handed a [`McpView`] snapshot each frame (the shell reads
//!   `crate::mcp::status()`), and a click is handed back to the shell, which
//!   owns the automation queue the server attaches to.
//!
//! Not to be confused with [`crate::panels::info_windows`], the pinned
//! per-ENTITY inspectors opened from the context bar. This window is about the
//! application; those are about a face, an edge or a solid.

use crate::automation::hit_keys::HitKeyDoc;
use crate::diagnostics::Diagnostics;
use eframe::egui;
use std::collections::HashMap;

/// The project licence, as authored. `BREP_docs` renders this same file as the
/// help site's first Licences page.
///
/// This is the crate's OWN copy, not the repository root's. An `include_str!`
/// that climbs out of the package compiles in a checkout and fails in the
/// tarball `cargo publish` verifies, which is exactly where nobody looks —
/// `build.rs` holds the two copies identical so the distinction stays
/// bookkeeping rather than a second source of truth.
const PROJECT_LICENCE: &str = include_str!("../../LICENSE.md");

/// The maintained third-party notices, as authored — the embedded fonts and the
/// material whose licence requires its notice to travel with it. The crate's own
/// copy, for the reason given above.
const THIRD_PARTY_NOTICES: &str = include_str!("../../THIRD-PARTY-NOTICES.md");

/// The generated crate inventory on the help site: every crate the application
/// ships, grouped under its declared licence expression. Generated from `cargo
/// metadata` by `BREP_docs` (see its `licences` module), so it exists only
/// beside the served page — hence a link rather than an include.
const INVENTORY_URL: &str = "help/licences/third-party-crates.html";

/// The label on the start button — the one a user looks for in the window.
pub const START_MCP_LABEL: &str = "Start MCP server";

/// What this build says when it has no server to start: main.rs's own words
/// for `--mcp` on such a build, so the button and the flag agree.
pub const NO_MCP_TEXT: &str = "this build has no MCP server (built without the `mcp` feature)";

/// What the "Connect an agent" section shows this frame. A SNAPSHOT: the
/// shell builds it from `crate::mcp::status()` (or [`McpView::Unavailable`]
/// when the build has no server) so this module compiles the same with and
/// without the `mcp` feature and on the web.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum McpView {
    /// No server in this build (built without `mcp`, or the browser build).
    Unavailable,
    /// Not running; `error` is the console's line from the last failure.
    Stopped { error: Option<String> },
    /// Serving: `instructions` is EXACTLY `crate::mcp::agent_instructions` for
    /// the live `url`; `note` says a fallback port was used, when one was.
    Running { url: String, instructions: String, note: Option<String> },
}

/// The Info window's state. Its whole model is `open`: everything it draws is
/// either a compiled-in constant or read live from the app's [`Diagnostics`]
/// and the [`McpView`] it is handed.
#[derive(Default)]
pub struct InfoPanel {
    /// Whether the window is showing — the toolbar ℹ toggle's flag, also set by
    /// the `info_window` command.
    pub open: bool,
    /// Per-frame widget screen rects for the headed verifier, like every other
    /// panel.
    hits: HashMap<String, egui::Rect>,
    /// Set by a click on **Start MCP server**; the shell takes it after the
    /// frame ([`Self::take_mcp_start`]) because the shell, not this window,
    /// owns the queue the server attaches to.
    mcp_start: bool,
}

impl InfoPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether **Start MCP server** was clicked since the last call. The shell
    /// answers it with `crate::mcp::start_from_window`.
    pub fn take_mcp_start(&mut self) -> bool {
        std::mem::take(&mut self.mcp_start)
    }

    /// Draw the floating window (if open) at ctx level, after the panels, so it
    /// floats over the shell. Idempotent while closed.
    pub fn show(&mut self, ctx: &egui::Context, diagnostics: &Diagnostics, mcp: &McpView) {
        if !self.open {
            return;
        }
        // `egui::Window::open` needs its own `&mut bool`; borrow a copy so the
        // draw closure can still take `&mut self`, then fold the close back in
        // (the Part Properties window's rule).
        let mut open = true;
        egui::Window::new("Info")
            .open(&mut open)
            .movable(true)
            .resizable(true)
            // A bounded default plus the filling ScrollArea in `body`: without a
            // filling child egui hugs the window to its content and the user
            // cannot drag it larger.
            .default_size([560.0, 520.0])
            // Clear of the Settings (620, 80) and Part Properties (660, 96)
            // rest positions, so opening all three does not stack them.
            .default_pos([120.0, 120.0])
            .show(ctx, |ui| self.body(ui, diagnostics, mcp));
        self.open = open;

        if crate::automation::registry::enabled() {
            crate::automation::registry::publish(
                "__brepInfoHit",
                "info window widget rects (copy, inventory, licence:*, mcp:*)",
                &self.hits_json(),
            );
        }
    }

    fn body(&mut self, ui: &mut egui::Ui, diagnostics: &Diagnostics, mcp: &McpView) {
        self.hits.clear();
        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .show(ui, |ui| {
                self.diagnostics_section(ui, diagnostics);
                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                self.mcp_section(ui, mcp);
                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                self.licences_section(ui);
            });
    }

    /// The button that starts the MCP server in this window, and — once it
    /// runs — the console's own agent instructions to copy into Claude Code
    /// or Codex. The button is the whole control: disabled with the reason
    /// when there is nothing to start (no server in this build, or it already
    /// runs — started here or by `--mcp`, the section cannot tell and need
    /// not), enabled otherwise. A failure's line sits under it in the words
    /// the console used, so a window launched from a desktop icon, with no
    /// terminal to read, still says what went wrong.
    fn mcp_section(&mut self, ui: &mut egui::Ui, mcp: &McpView) {
        ui.heading("Connect an agent (MCP)");
        ui.label(
            egui::RichText::new(
                "Start the MCP server inside this window — what launching with --mcp does — so \
                 Claude Code or Codex can drive the documents you see here.",
            )
            .weak()
            .small(),
        );
        ui.add_space(4.0);
        let (enabled, label) = match mcp {
            McpView::Unavailable => (false, START_MCP_LABEL),
            McpView::Stopped { .. } => (true, START_MCP_LABEL),
            McpView::Running { .. } => (false, "MCP server running"),
        };
        let start = ui.add_enabled(enabled, egui::Button::new(label));
        self.hit("mcp:start", &start);
        if start.clicked() {
            self.mcp_start = true;
        }
        match mcp {
            McpView::Unavailable => {
                ui.label(egui::RichText::new(NO_MCP_TEXT).weak());
            }
            McpView::Stopped { error: None } => {}
            McpView::Stopped { error: Some(error) } => {
                ui.label(egui::RichText::new(error).color(ui.visuals().error_fg_color));
            }
            McpView::Running { url, instructions, note } => {
                ui.label(format!("Listening on {url}"));
                if let Some(note) = note {
                    ui.label(egui::RichText::new(note).color(ui.visuals().warn_fg_color));
                }
                ui.add_space(4.0);
                ui.label(
                    egui::RichText::new("Paste this into your agent (the same text the terminal prints):")
                        .weak()
                        .small(),
                );
                // A read-only text edit: selectable with the pointer, wrapped
                // to the window, and painted as ONE galley so a test reading
                // the frame sees the whole text, not a line of it.
                ui.add(
                    egui::TextEdit::multiline(&mut instructions.as_str())
                        .font(egui::TextStyle::Monospace)
                        .desired_width(f32::INFINITY)
                        .desired_rows(instructions.lines().count()),
                );
                let copy = ui.button("Copy agent instructions");
                self.hit("mcp:copy", &copy);
                if copy.clicked() {
                    ui.ctx().copy_text(instructions.clone());
                }
            }
        }
    }

    /// The diagnostics grid, then the one button that puts the very same text a
    /// problem report would carry on the clipboard — for a user reporting
    /// somewhere other than the in-app form.
    fn diagnostics_section(&mut self, ui: &mut egui::Ui, diagnostics: &Diagnostics) {
        ui.heading("Diagnostics");
        ui.label(
            egui::RichText::new("Sent with every problem report from the Submit Bug button.")
                .weak()
                .small(),
        );
        ui.add_space(4.0);
        egui::Grid::new("brep-info-diagnostics")
            .num_columns(2)
            .spacing([12.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                for (label, value) in diagnostics.rows() {
                    ui.label(egui::RichText::new(label).strong());
                    ui.label(egui::RichText::new(value).monospace());
                    ui.end_row();
                }
            });
        ui.add_space(6.0);
        let copy = ui.button("Copy diagnostics");
        self.hit("copy", &copy);
        if copy.clicked() {
            ui.ctx().copy_text(diagnostics.report_text());
        }
    }

    /// The licences: the project's own, the maintained notices, and a link to
    /// the generated crate inventory. Each authored file is behind a collapsing
    /// header — several thousand words of licence text opened by default would
    /// bury the diagnostics above it.
    fn licences_section(&mut self, ui: &mut egui::Ui) {
        ui.heading("Licences");
        ui.add_space(4.0);
        let project = egui::CollapsingHeader::new("Project licence (LICENSE.md)")
            .id_salt("brep-info-licence-project")
            .show(ui, |ui| licence_text(ui, PROJECT_LICENCE));
        self.hit("licence:project", &project.header_response);
        let notices = egui::CollapsingHeader::new("Third-party notices (THIRD-PARTY-NOTICES.md)")
            .id_salt("brep-info-licence-notices")
            .show(ui, |ui| licence_text(ui, THIRD_PARTY_NOTICES));
        self.hit("licence:notices", &notices.header_response);
        ui.add_space(6.0);
        ui.label(
            egui::RichText::new(
                "Every crate this application ships, grouped by its licence, is generated \
                 into the help site from the dependency graph itself:",
            )
            .weak()
            .small(),
        );
        let inventory = ui.button("Open the third-party crate inventory");
        self.hit("inventory", &inventory);
        if inventory.clicked() {
            ui.ctx().open_url(egui::OpenUrl::new_tab(INVENTORY_URL));
        }
    }

    /// Record a widget's screen rect for the headed verifier.
    fn hit(&mut self, key: &str, resp: &egui::Response) {
        self.hits.insert(key.to_string(), resp.rect);
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        let map: serde_json::Map<String, serde_json::Value> = self
            .hits
            .iter()
            .map(|(k, r)| {
                (
                    k.clone(),
                    serde_json::json!([r.center().x, r.center().y, r.width(), r.height()]),
                )
            })
            .collect();
        serde_json::Value::Object(map).to_string()
    }
}

/// One authored licence file, verbatim. Markdown as typed — the app has no
/// markdown renderer, and a licence is one of the few texts where showing
/// exactly the bytes that ship is the right answer anyway. Selectable, so a
/// reader can copy a clause out.
fn licence_text(ui: &mut egui::Ui, text: &str) {
    ui.add(egui::Label::new(egui::RichText::new(text).monospace().small()).wrap());
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
/// Every `command` here is `None`, and honestly so: the registered command is
/// the one that DOES what a click does, and no command copies to a clipboard,
/// opens this particular help page or expands a header. The window itself is
/// reachable without a pointer (`info_window`), and the diagnostics it shows are
/// readable without one (`diagnostics`) — naming either against a button that
/// does something else would put a promise in the generated widget docs that
/// `tools/call` cannot keep.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "info", prefix: "copy", meaning: "copy the diagnostics block to the clipboard (the same rows `diagnostics` returns)", command: None },
    HitKeyDoc { panel: "info", prefix: "inventory", meaning: "open the generated third-party crate inventory on the help site", command: None },
    HitKeyDoc { panel: "info", prefix: "licence:", meaning: "expand a licence text (licence:project, licence:notices)", command: None },
    // No command either: a server started from the agent's own tool call
    // would be the server that call came in on, and the copy goes to a
    // clipboard. The instructions are the console's, so a host has them.
    HitKeyDoc { panel: "info", prefix: "mcp:", meaning: "the Connect an agent section: mcp:start starts the embedded MCP server (disabled while it runs or in a build without one), mcp:copy copies the agent instructions it then shows", command: None },
];

