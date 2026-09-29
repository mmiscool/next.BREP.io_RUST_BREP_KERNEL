//! Display-settings panel — the schema-driven settings form. Drawn as a
//! FLOATING window (movable + resizable [`egui::Window`], toggled from the
//! toolbar gear ⚙ button), mirroring the Properties window: a `pub open` flag the
//! toolbar binds + a ctx-level `show(&mut self, ctx, state, store)` the shell
//! calls after the panels. The panel OWNS only its transient UI state (which
//! nodes are open); `EngineState` stays the single brain, borrowed in.
//!
//! # Model colours are NOT set here
//!
//! A body's or face's colour is a durable `color` METADATA attribute, set in the
//! Info window and saved with the document. This panel carries only the display
//! switch over it — `Faces ▸ Override model colors`, an ordinary schema `Bool`
//! that makes the viewport ignore those colours without touching them. The old
//! `Per-Solid Colors` tab wrote a transient override that no document ever
//! stored and any feature edit threw away; it is gone.
//!
//! # One TAB per section, each drawn as a tree
//!
//! The window opens on a tab strip ([`Tab`], the Info window's `selectable_value`
//! strip) — `Display` / `Assemblies` — and draws exactly ONE section below it. Each section is still the SAME connector-line `[+]/[-]` tree
//! the feature history and Scene panels use (the shared [`tree`] node helper), so
//! the whole app reads as one system:
//!   * `Display` → `[-] Display settings` (root) → one collapsible BRANCH per
//!     schema group (`Scene`, `Faces`, `Edges`, …) → one LEAF per field, whose
//!     node label is the field label and whose right-aligned content is the field
//!     input ([`form::field_input`], EXACTLY like the feature tree's
//!     `schema_field`).
//!   * `Assemblies` → `[-] Assemblies` (root) → the BOM column configuration.
//! Group open-state is tracked on the panel (default open). Every ROOT defaults
//! OPEN too: the roots that used to default collapsed did so only because they
//! shared one scroll — a tab whose entire content is one `[+]` row is not
//! worth the click.
//!
//! Only the ACTIVE tab's widget rects are published to `__brepSettingsHit`, since
//! `hits` is rebuilt each frame from what was actually drawn.

use crate::automation::hit_keys::HitKeyDoc;
use crate::form;
use crate::panels::bom_columns;
use crate::panels::tree::{self, TreeRow};
use crate::store::{ModelStore, SETTINGS_KEY};
use brep_render::engine_state::EngineState;
use brep_render::style::{settings_form_fields, FormField, RenderSettings};
use eframe::egui;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

/// The amber a BOM-column parse problem is listed in — the status map's
/// warning amber, the same one the structure tree's outdated badge uses. A
/// problem is a note about ONE line, not a failure, so it is not error red.
const PROBLEM_AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0x9f, 0x0a);

/// The tabs of the Settings window — one per section. Each draws its own tree;
/// nothing is shared between them but the window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    /// The schema-driven render settings.
    Display,
    /// Assembly-wide configuration (today: the BOM's columns).
    Assemblies,
    /// The PLM this session is on or configured for: sign-in, check, forget
    /// (plan S1 part 3). Present only when the store reports a PLM.
    Plm,
}

/// The display-settings panel's own transient UI state. It holds NO model state:
/// the settings buffer is re-seeded from the live engine each frame (see
/// [`SettingsPanel::settings_section`]).
pub struct SettingsPanel {
    /// Whether the floating window is shown. Toggled by the toolbar gear button
    /// and by the window's own close (`×`) button; public so the toolbar can bind
    /// it.
    pub open: bool,
    /// Per-frame egui widget screen rects (keyed `tab:<name>` / `field:<key>` /
    /// `group:<name>` / …), published to JS for the headed verifier. Rebuilt every
    /// frame, so it holds only the ACTIVE tab's widgets.
    hits: HashMap<String, egui::Rect>,
    /// Which tab is shown. Defaults to `Display`, the tab the window has always
    /// opened on.
    tab: Tab,
    /// The `Display settings` root is collapsed (false = open — it defaults open).
    display_collapsed: bool,
    /// Setting GROUPS explicitly COLLAPSED, by group name (absent = open — groups
    /// default open, matching the retired per-group CollapsingHeaders).
    closed_groups: HashSet<String>,
    /// The `Assemblies` root is collapsed (false = open — see above).
    assemblies_collapsed: bool,
    /// The BOM-columns textarea's live edit buffer. Held here, not re-seeded
    /// per frame like the settings JSON, because a multi-line editor cannot be
    /// re-seeded mid-edit without fighting the caret. It tracks the engine
    /// while UNFOCUSED and commits on focus-loss (the expressions editor's
    /// rule); `None` = not yet seeded.
    bom_columns_buf: Option<String>,
    /// The PLM tab's state, built when the store first reports a PLM and
    /// rebuilt if its URL changes. `None` in a file-only session.
    plm: Option<crate::plm::connection::PlmConnection>,
    /// Set by the app every frame: why this session cannot switch to the PLM
    /// live (open named or dirty documents), or `None` when it can.
    pub reconnect_blocker: Option<String>,
    /// Connect now was pressed: the config root the app switches to.
    reconnect_request: Option<std::path::PathBuf>,
}

impl SettingsPanel {
    /// Open the window on its PLM tab ("Open in CAD" before signing in): the
    /// tab exists while the store reports a PLM, signed in or not.
    pub fn show_plm_tab(&mut self) {
        self.open = true;
        self.tab = Tab::Plm;
    }

    /// A fresh panel. The settings buffer is re-seeded from the engine every frame
    /// (not stored), so construction needs no engine handle.
    pub fn new() -> Self {
        Self {
            open: false,
            hits: HashMap::new(),
            tab: Tab::Display,
            display_collapsed: false,
            closed_groups: HashSet::new(),
            assemblies_collapsed: false,
            bom_columns_buf: None,
            plm: None,
            reconnect_blocker: None,
            reconnect_request: None,
        }
    }

    /// Draw the floating window (if open) at ctx level — after the panels, like
    /// the file dialog, so it floats over the shell. The `open` flag is shared with
    /// the toolbar gear button (which toggles it) and the window's own `×` (which
    /// closes it). `EngineState` is the single brain, borrowed in.
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState, store: &dyn ModelStore) {
        if self.open {
            // `egui::Window::open` needs its own `&mut bool`; borrow a copy so the
            // draw closure can still take `&mut self`, then fold the close back in.
            let mut open = true;
            egui::Window::new("Settings")
                .open(&mut open)
                .movable(true)
                .resizable(true)
                // A bounded default size + a fill ScrollArea (in `body`, under the
                // tab strip) makes the window FREELY resizable LARGER than its
                // content: without a filling child egui hugs the window to content
                // and won't grow.
                .default_size([320.0, 400.0])
                // Rest on the right so it floats clear of the left panel; the user
                // can drag it anywhere.
                .default_pos([720.0, 56.0])
                .show(ctx, |ui| self.body(ui, state, store));
            self.open = open;
        } else {
            // A CLOSED window publishes no rects. The publish used to sit inside
            // the branch above, so the last open frame's map stayed in the
            // registry for the rest of the session: `hit_rects settings/` kept
            // answering with fifty fields of a window that was not on screen, and
            // `click_widget` on one of them aimed a click at empty space. Same
            // rule as the workbench strip and the sketch overlay, which publish an
            // empty map while they are hidden.
            self.hits.clear();
        }

        // Publish this frame's widget rects for the headed verifier (parity
        // with the history + scene panels) — empty while the window is closed.
        if crate::automation::registry::enabled() {
            crate::automation::registry::publish("__brepSettingsHit", "settings window widget rects (field:*, group:*, panel:clip); empty while the window is closed", &self.hits_json());
            crate::automation::registry::publish("__brepPlm", "the Settings PLM tab: {tab, status, url, username, reason, busy, outcome, failed}; tab false (and the rest null) in a file-only session", &self.plm_json(store));
        }
    }

    /// The window body: the tab strip, then the ONE section that tab selects —
    /// each still built on the shared [`tree`] node helper, so every tab reads as
    /// the same tree the history + scene panels draw.
    ///
    /// The strip sits ABOVE the ScrollArea (rather than inside the one `show` used
    /// to wrap the whole body in), so the tabs stay put while a long settings tree
    /// scrolls under them.
    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState, store: &dyn ModelStore) {
        self.hits.clear();
        // The window's VISIBLE region, under the same key every scrolling pane
        // publishes. The Display tree is longer than the window, and egui hands
        // out a layout rect for a row that is scrolled past the bottom edge — a
        // click there lands outside the window's clip and does nothing at all,
        // which reads as "the switch is broken" rather than "the switch is off
        // screen". With the region published, `scroll_into_view` wheels the row
        // up first. Taken HERE rather than inside the scroll area so the tab
        // strip above it counts as visible too: a clip that excluded the strip
        // would send a click on `tab:assemblies` scrolling forever looking for
        // a widget that never moves.
        self.hits.insert("panel:clip".into(), ui.clip_rect());

        // The tab drawn this frame is the one that was active BEFORE the strip.
        // `selectable_value` switches `self.tab` mid-frame, and a section that
        // vanishes the same frame it loses the click never gets its focus-loss —
        // which is how the Assemblies editor COMMITS. Deferring by one frame lets
        // the editor blur normally (invisible at 60fps, and the difference between
        // "my BOM columns saved" and "my typing disappeared").
        // The PLM tab exists only while the store reports a PLM: a file-only
        // session has no such tab at all.
        self.sync_plm(store.plm_session());
        if self.plm.is_none() && self.tab == Tab::Plm {
            self.tab = Tab::Display;
        }
        let tab = self.tab;
        ui.horizontal(|ui| {
            let display = ui.selectable_value(&mut self.tab, Tab::Display, "Display");
            let assemblies = ui.selectable_value(&mut self.tab, Tab::Assemblies, "Assemblies");
            self.hits.insert("tab:display".into(), display.rect);
            self.hits.insert("tab:assemblies".into(), assemblies.rect);
            if self.plm.is_some() {
                let plm = ui.selectable_value(&mut self.tab, Tab::Plm, "PLM");
                self.hits.insert("tab:plm".into(), plm.rect);
            }
        });
        // ...and because of that defer, the switch needs ONE more frame to show
        // its new section. The shell only requests a repaint while work is
        // pending (`app.rs`: a run / queries / mesh imports), so on native a
        // click with no further input would leave the strip highlighting a tab
        // whose content has not been drawn yet. Ask for that frame here.
        if self.tab != tab {
            ui.ctx().request_repaint();
        }
        ui.separator();

        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                // Tight, tree-like row spacing so connector verticals read
                // continuously — the same the history + scene trees set (this panel
                // must match them). Set INSIDE the scroll so the tab strip above
                // keeps ordinary widget spacing.
                ui.spacing_mut().item_spacing.y = 2.0;
                match tab {
                    Tab::Display => self.settings_section(ui, state, store),
                    Tab::Assemblies => self.assemblies_section(ui, state, store),
                    Tab::Plm => self.plm_section(ui),
                }
            });
    }

    /// The `__brepPlm` state. Read from the store whether or not the window is
    /// open, so a script can see the session's PLM without opening Settings.
    fn plm_json(&mut self, store: &dyn ModelStore) -> String {
        use crate::plm::connection::{Outcome, Status};
        self.sync_plm(store.plm_session());
        let Some(plm) = self.plm.as_mut() else {
            return serde_json::json!({ "tab": false, "status": null }).to_string();
        };
        plm.poll();
        let (status, url, username, reason) = match &plm.status {
            Status::Connected { url, username } => ("connected", url.clone(), Some(username.clone()), None),
            Status::NotConnected { url, reason } => ("not-connected", url.clone(), None, reason.clone()),
        };
        let (outcome, failed) = match &plm.last {
            Some(Outcome::Done(text)) => (Some(text.clone()), false),
            Some(Outcome::Failed(text)) => (Some(text.clone()), true),
            None => (None, false),
        };
        serde_json::json!({
            "tab": true, "status": status, "url": url, "username": username, "reason": reason,
            "busy": plm.busy(), "outcome": outcome, "failed": failed,
            "connectable": plm.connectable(), "reconnectBlocker": self.reconnect_blocker,
        })
        .to_string()
    }

    /// Connect now's request, for the app to act on (once).
    pub fn take_reconnect_request(&mut self) -> Option<std::path::PathBuf> {
        self.reconnect_request.take()
    }

    /// What the app's switch did: shown in the tab as its outcome.
    pub fn reconnected(&mut self, outcome: Result<String, String>) {
        if let Some(plm) = self.plm.as_mut() {
            plm.reconnected(outcome);
        }
    }

    /// Keep the PLM tab's state in step with what the store reports.
    fn sync_plm(&mut self, session: Option<crate::plm::connection::Session>) {
        use crate::plm::connection::PlmConnection;
        let Some(session) = session else {
            self.plm = None;
            return;
        };
        let url = match &session.status {
            crate::plm::connection::Status::Connected { url, .. } | crate::plm::connection::Status::NotConnected { url, .. } => url.clone(),
        };
        let same = self.plm.as_ref().is_some_and(|c| c.url() == url && *c.keep() == session.keep);
        match self.plm.as_mut().filter(|_| same) {
            // Same server, same keep: a live switch moves only the status.
            Some(plm) => plm.set_status(session.status),
            None => self.plm = Some(PlmConnection::new(session.status, session.keep)),
        }
    }

    /// The `PLM` TAB (plan S1 part 3): where this session's PLM is and who it
    /// is signed in as, or why it is not; a token paste and a password
    /// sign-in; a check; and Forget. Hit keys `plm:*`.
    fn plm_section(&mut self, ui: &mut egui::Ui) {
        use crate::plm::connection::{Keep, Outcome, Status};
        let Some(plm) = self.plm.as_mut() else { return };
        if plm.poll() || plm.busy() {
            ui.ctx().request_repaint();
        }
        if plm.take_reload() {
            reload_page();
        }
        let hits = &mut self.hits;
        let status = match &plm.status {
            Status::Connected { url, username } => format!("Connected to {url} as {username}."),
            Status::NotConnected { url, reason: None } => format!("Not connected to {url}."),
            Status::NotConnected { url, reason: Some(why) } => format!("Not connected to {url}: {why}"),
        };
        let shown = ui.add(egui::Label::new(status).wrap());
        hits.insert("plm:status".into(), shown.rect);
        ui.separator();

        let native = matches!(plm.keep(), Keep::Files(_));
        let idle = !plm.busy();
        if native {
            ui.label("Paste an API token (from the PLM's web page):");
            let token = ui.add(egui::TextEdit::singleline(&mut plm.token_input).password(true).hint_text("plm_…"));
            hits.insert("plm:token".into(), token.rect);
            let use_token = ui.add_enabled(idle && !plm.token_input.trim().is_empty(), egui::Button::new("Use token"));
            hits.insert("plm:use-token".into(), use_token.rect);
            if use_token.clicked() {
                plm.use_token();
            }
            ui.separator();
        }
        ui.label(if native { "Or sign in, and this machine keeps a token (not the password):" } else { "Sign in to the PLM:" });
        let username = ui.add(egui::TextEdit::singleline(&mut plm.username).hint_text("username"));
        hits.insert("plm:username".into(), username.rect);
        let password = ui.add(egui::TextEdit::singleline(&mut plm.password).password(true).hint_text("password"));
        hits.insert("plm:password".into(), password.rect);
        let can_sign_in = idle && !plm.username.trim().is_empty() && !plm.password.is_empty();
        let sign_in = ui.add_enabled(can_sign_in, egui::Button::new("Sign in"));
        hits.insert("plm:sign-in".into(), sign_in.rect);
        if sign_in.clicked() {
            plm.use_password();
        }
        ui.separator();
        // Connect now (native): once the server has taken this machine's
        // sign-in, switch this session to it live instead of at next start
        // (only in the state a fresh start has: no named or dirty document).
        if native && matches!(plm.status, Status::NotConnected { .. }) && plm.connectable() {
            let blocked = self.reconnect_blocker.clone();
            let connect = ui.add_enabled(idle && blocked.is_none(), egui::Button::new("Connect now"));
            hits.insert("plm:connect".into(), connect.rect);
            if let Some(why) = &blocked {
                ui.label(egui::RichText::new(why).color(PROBLEM_AMBER));
            }
            if connect.clicked() {
                if let Keep::Files(root) = plm.keep() {
                    self.reconnect_request = Some(root.clone());
                }
            }
        }
        ui.horizontal(|ui| {
            let check = ui.add_enabled(idle, egui::Button::new("Check connection"));
            hits.insert("plm:check".into(), check.rect);
            if check.clicked() {
                plm.check();
            }
            let forget = ui.add_enabled(idle, egui::Button::new(if native { "Forget this machine's sign-in" } else { "Sign out" }));
            hits.insert("plm:forget".into(), forget.rect);
            if forget.clicked() {
                plm.forget();
            }
        });
        if !idle {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Asking the server…");
            });
        }
        if let Some(outcome) = &plm.last {
            let (text, color) = match outcome {
                Outcome::Done(text) => (text.clone(), ui.visuals().text_color()),
                Outcome::Failed(text) => (text.clone(), PROBLEM_AMBER),
            };
            let said = ui.add(egui::Label::new(egui::RichText::new(text).color(color)).wrap());
            hits.insert("plm:outcome".into(), said.rect);
        }
    }

    /// The `Assemblies` TAB: the BOM's COLUMN CONFIGURATION as one multiline
    /// textarea, one column per line, a leading `*` for shown.
    ///
    /// A hand-written root rather than a schema field: the schema's `FieldKind`s
    /// are all
    /// single-widget and render into a tree row's RIGHT-ALIGNED content slot,
    /// which is exactly the wrong place for a full-width multi-line editor.
    /// Adding a `TextArea` variant would also force both exhaustive
    /// `FieldKind` matches open for one consumer.
    ///
    /// Commits on focus-LOSS — which includes LEAVING THE TAB, since [`body`]
    /// draws the pre-click tab for one more frame so this editor is still on
    /// screen to blur ([`SettingsPanel::body`]) — not per keystroke: this text is
    /// persisted to the store on commit, and on native that is a rewrite of
    /// `~/.config/brep-app/settings.json` — per character is a file write per
    /// character. Parse problems are listed under the editor, naming the line,
    /// and the text is never rewritten by the panel: a typo costs one column,
    /// not the configuration.
    fn assemblies_section(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) {
        let open = !self.assemblies_collapsed;
        let root_resp = tree::node(
            ui,
            TreeRow {
                guides: &[],
                is_last: true,
                expandable: true,
                expanded: open,
                root: true,
                glyph: None,
                label: "Assemblies",
                selected: false,
                highlighted: false,
                draggable: false,
                tint: None,
            },
            |_| {},
        );
        self.hits.insert("box:__assemblies".into(), root_resp.box_rect);
        if root_resp.toggled || root_resp.clicked() {
            self.assemblies_collapsed = !self.assemblies_collapsed;
        }
        if !open {
            // Drop the buffer while closed so the next open re-seeds from the
            // engine (a BOM header drag rewrites this text behind the panel).
            self.bom_columns_buf = None;
            return;
        }

        // Empty stored text means "the shipped default", so the editor shows
        // the default rather than a blank box the user has to guess at.
        let stored = bom_columns::effective_text(&state.settings.bom_columns);
        let buffer = self.bom_columns_buf.get_or_insert_with(|| stored.clone());

        ui.label(egui::RichText::new("BOM columns").strong());
        ui.label(
            egui::RichText::new(
                "One per line, in order. A leading * shows it. \
                 part.<Field> is stored on the part, occurrence.<Field> on one placement. \
                 A line that is just - freezes the columns above it; the rest scroll.",
            )
            .weak(),
        );
        let editor = ui.add(
            egui::TextEdit::multiline(buffer)
                .id_salt("bom-columns-editor")
                .desired_rows(8)
                .desired_width(f32::INFINITY)
                .code_editor(),
        );
        self.hits.insert("field:bomColumns".into(), editor.rect);

        if editor.lost_focus() {
            // Commit: store the text VERBATIM (never the parse's idea of it).
            let mut settings_json: serde_json::Value =
                serde_json::from_str(&state.settings_json()).unwrap_or(serde_json::Value::Null);
            if let Some(object) = settings_json.as_object_mut() {
                object.insert(
                    "bomColumns".into(),
                    serde_json::Value::String(buffer.clone()),
                );
                let json = settings_json.to_string();
                let _ = state.apply_settings_json(&json);
                let _ = store.write(SETTINGS_KEY, &json);
            }
        } else if !editor.has_focus() && *buffer != stored {
            // Unfocused and out of step with the engine — the BOM's own header
            // drag rewrote the configuration. Track it rather than showing a
            // stale copy the next commit would write back.
            *buffer = stored;
        }

        // Parse problems, by line. Listed rather than thrown: the text stands
        // exactly as typed and every other line still works.
        let parsed = bom_columns::parse(buffer);
        if parsed.problems.is_empty() {
            ui.label(
                egui::RichText::new(format!(
                    "{} columns, {} shown",
                    parsed.columns.len(),
                    parsed.columns.iter().filter(|column| column.shown).count()
                ))
                .weak(),
            );
        } else {
            for problem in &parsed.problems {
                ui.label(egui::RichText::new(problem).color(PROBLEM_AMBER));
            }
        }
        let reset = ui.button("Reset BOM columns");
        self.hits.insert("bom-columns:reset".into(), reset.rect);
        if reset.clicked() {
            self.bom_columns_buf = None;
            let mut settings_json: serde_json::Value =
                serde_json::from_str(&state.settings_json()).unwrap_or(serde_json::Value::Null);
            if let Some(object) = settings_json.as_object_mut() {
                // Back to EMPTY, which means "the shipped default" — so a later
                // change to that default still reaches this user.
                object.insert("bomColumns".into(), serde_json::Value::String(String::new()));
                let json = settings_json.to_string();
                let _ = state.apply_settings_json(&json);
                let _ = store.write(SETTINGS_KEY, &json);
            }
        }
        ui.add_space(4.0);
    }

    /// The schema-driven display-settings TREE: a `Display settings` root, one
    /// collapsible branch per schema group, one leaf per field. Any edit applies to
    /// `EngineState` (bumps `settings_generation` + `dirty`, so the GPU refreshes)
    /// and persists through the storage seam.
    fn settings_section(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) {
        // Re-seed a per-frame LOCAL buffer from the LIVE engine settings BEFORE
        // rendering. The apply below writes the WHOLE buffer, so a buffer kept
        // across frames would clobber every setting changed elsewhere (the toolbar
        // wireframe / projection toggles) back to a stale snapshot — the "changing
        // Render Quality resets my wireframe" bug. A fresh local each frame makes
        // external changes authoritative and keeps untouched fields a no-op
        // round-trip (`apply_json`/`to_json` are a documented identity).
        let before = state.settings_json();
        let mut settings_json: Value = serde_json::from_str(&before).unwrap_or(Value::Null);
        let fields = settings_form_fields();

        // Group the schema's contiguous same-group runs, preserving order (the
        // schema lists each group's fields together).
        let mut groups: Vec<(String, Vec<&FormField>)> = Vec::new();
        for f in &fields {
            if let Some(g) = groups.iter_mut().find(|(n, _)| *n == f.group) {
                g.1.push(f);
            } else {
                groups.push((f.group.clone(), vec![f]));
            }
        }

        // --- ROOT: `[-] Display settings` (defaults open) ---------------------
        let root_open = !self.display_collapsed;
        let root_resp = tree::node(
            ui,
            TreeRow {
                guides: &[],
                is_last: true,
                expandable: true,
                expanded: root_open,
                root: true,
                glyph: None,
                label: "Display settings",
                selected: false,
                highlighted: false,
                draggable: false,
                tint: None,
            },
            |_| {},
        );
        if root_resp.toggled || root_resp.clicked() {
            self.display_collapsed = !self.display_collapsed;
        }

        let mut changed = false;
        if root_open {
            let n = groups.len();
            for (gi, (gname, gfields)) in groups.iter().enumerate() {
                let is_last = gi + 1 == n;
                let open = !self.closed_groups.contains(gname);
                let resp = tree::node(ui, TreeRow::branch(&[], is_last, open, gname), |_| {});
                self.hits.insert(format!("group:{gname}"), resp.box_rect);
                if resp.toggled || resp.clicked() {
                    if open {
                        self.closed_groups.insert(gname.clone());
                    } else {
                        self.closed_groups.remove(gname);
                    }
                }
                if !open {
                    continue;
                }
                let base = tree::child_guides(&[], is_last);
                let m = gfields.len();
                for (fi, &f) in gfields.iter().enumerate() {
                    changed |= self.settings_leaf(ui, f, &mut settings_json, &base, fi + 1 == m);
                }
            }
        }

        // Commit the whole buffer ONCE on any edit (same apply + persist path as
        // before), so the engine re-runs / the GPU refreshes exactly as it did.
        //
        // ...and only when the settings actually MOVED. Some field reports a
        // change on every frame the tree is drawn, and the write it caused went
        // unnoticed on a file store (one settings.json rewrite per frame) until
        // the PLM made each one a request: with the window open the store was
        // never idle. Comparing through the engine's own serialisation makes a
        // no-op edit a no-op whichever widget claims otherwise.
        if changed {
            let json = settings_json.to_string();
            let _ = state.apply_settings_json(&json);
            if state.settings_json() != before {
                let _ = store.write(SETTINGS_KEY, &json);
            }
        }

        // Reset to defaults — only while the display root is OPEN, matching the
        // retired CollapsingHeader that hid it when the section was collapsed. It
        // resets the DISPLAY settings only, which is why it belongs to this tab.
        if root_open {
            ui.add_space(2.0);
            if ui.button("Reset to defaults").clicked() {
                // Full reset: rebase to defaults, then apply the serialized defaults
                // (so every key returns, not just the overridden ones) + persist.
                state.settings = RenderSettings::default();
                let json = state.settings.to_json();
                let _ = state.apply_settings_json(&json);
                let _ = store.write(SETTINGS_KEY, &json);
            }
        }
    }

    /// Render one settings field as a tree LEAF: the field label is the node label;
    /// its input widget ([`form::field_input`]) fills the row's RIGHT-aligned
    /// content, exactly like the feature tree's `schema_field`. Settings keys are
    /// unique across the schema, so no id-stack scoping is needed. Returns whether
    /// the field changed (the caller commits the whole buffer once).
    fn settings_leaf(
        &mut self,
        ui: &mut egui::Ui,
        field: &FormField,
        current: &mut Value,
        guides: &[bool],
        is_last: bool,
    ) -> bool {
        let mut changed = false;
        let mut rect = egui::Rect::NOTHING;
        tree::node(ui, TreeRow::leaf(guides, is_last, &field.label), |ui| {
            // The tree row's content area is RIGHT-aligned (`right_to_left`), so the
            // input sits at the panel edge with the label on the left — the feature
            // tree's exact placement, and the layout `field_input` reads to keep its
            // inputs COMPACT here. Settings have no reference / button fields, so
            // `field_input`'s click sink is `None`.
            let (ch, r) = form::field_input(ui, field, current, None, &mut form::FieldActions::default());
            changed = ch;
            rect = r;
        });
        self.hits.insert(format!("field:{}", field.key()), rect);
        changed
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}


/// Reload the page onto the PLM session a web sign-in (or sign-out) made.
fn reload_page() {
    #[cfg(target_arch = "wasm32")]
    if let Some(window) = web_sys::window() {
        let _ = window.location().reload();
    }
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "settings", prefix: "field:", meaning: "a settings field by key", command: None },
    HitKeyDoc { panel: "settings", prefix: "group:", meaning: "a settings group header", command: None },
    HitKeyDoc { panel: "settings", prefix: "tab:", meaning: "a settings tab (tab:display, tab:assemblies, and tab:plm while a PLM is configured)", command: None },
    HitKeyDoc { panel: "settings", prefix: "plm:", meaning: "the PLM tab: status, token, use-token, username, password, sign-in, check, forget, outcome", command: None },
    HitKeyDoc { panel: "settings", prefix: "box:", meaning: "expand/collapse a section", command: None },
    HitKeyDoc { panel: "settings", prefix: "bom-columns:reset", meaning: "reset the BOM columns", command: None },
    HitKeyDoc { panel: "settings", prefix: "panel:clip", meaning: "the visible region of the window", command: None },
];
