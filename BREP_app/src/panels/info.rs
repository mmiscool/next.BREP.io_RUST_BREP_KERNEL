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

/// The Info window's state. Its whole model is `open`: everything it draws is
/// either a compiled-in constant or read live from the app's [`Diagnostics`].
#[derive(Default)]
pub struct InfoPanel {
    /// Whether the window is showing — the toolbar ℹ toggle's flag, also set by
    /// the `info_window` command.
    pub open: bool,
    /// Per-frame widget screen rects for the headed verifier, like every other
    /// panel.
    hits: HashMap<String, egui::Rect>,
}

impl InfoPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the floating window (if open) at ctx level, after the panels, so it
    /// floats over the shell. Idempotent while closed.
    pub fn show(&mut self, ctx: &egui::Context, diagnostics: &Diagnostics) {
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
            .show(ctx, |ui| self.body(ui, diagnostics));
        self.open = open;

        if crate::automation::registry::enabled() {
            crate::automation::registry::publish(
                "__brepInfoHit",
                "info window widget rects (copy, inventory, licence:*)",
                &self.hits_json(),
            );
        }
    }

    fn body(&mut self, ui: &mut egui::Ui, diagnostics: &Diagnostics) {
        self.hits.clear();
        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .show(ui, |ui| {
                self.diagnostics_section(ui, diagnostics);
                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                self.licences_section(ui);
            });
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
];

