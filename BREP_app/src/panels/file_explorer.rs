//! Reusable, embeddable file explorer for the application's persistent store.
//! It owns only transient browsing state; callers decide what activating a file
//! means (open a document, choose a Save-As name, or insert an ACOMP part).
//!
//! Interaction model (matches a conventional file dialog): a single click on a
//! FILE row selects it (highlight + selection preview); a double-click, the
//! Enter key, or the footer confirm button *activates* it. Directories are pure
//! navigation — a single click walks into them. A left sidebar offers quick
//! places + pinned folders, and the top bar is a clickable breadcrumb with an
//! editable path and back / forward history — all driven through the
//! [`ModelStore`] seam so web and native share one UI.
//!
//! The explorer never sizes itself: it FILLS the region its host hands it. A
//! modal host wraps its whole body in [`dialog_body`], which puts one resize
//! grip at the dialog window's bottom-right corner; a window host ([`egui::Window`])
//! already has that grip. Either way the host's own trailing widgets go in a
//! [`dialog_footer`] BEFORE the explorer, so they pin to the bottom and the
//! file list is the only part that scrolls.

use crate::automation::hit_keys::HitKeyDoc;
use crate::store::{BrowserEntry, BrowserPlace, ModelStore, PlaceKind, PINNED_KEY};
use eframe::egui;
use egui_extras::{Column, TableBuilder};
use std::cmp::Ordering;

#[derive(Clone, Copy)]
pub struct FileExplorerOptions<'a> {
    pub hit_prefix: &'a str,
    pub empty_label: &'a str,
    pub row_icon: &'a str,
    pub current: Option<&'a str>,
    pub allow_delete: bool,
    pub allow_import: bool,
    pub import_label: &'a str,
    pub import_hit: &'a str,
    pub show_cancel: bool,
    /// Footer primary/confirm button label (`"Open"` / `"Import"` / `"Insert"`).
    /// `None` when the caller supplies its own action button and uses the
    /// explorer only for navigation + name selection (Save As, step-parts
    /// destination) — then keyboard confirm is disabled too, leaving Enter to
    /// the caller's name field.
    pub confirm_label: Option<&'a str>,
    pub extensions: &'a [&'a str],
}

impl<'a> FileExplorerOptions<'a> {
    pub fn open(current: Option<&'a str>) -> Self {
        Self {
            hit_prefix: "open",
            empty_label: "(no saved models)",
            row_icon: "\u{1F5CE}",
            current,
            allow_delete: true,
            allow_import: false,
            import_label: "Upload\u{2026}",
            import_hit: "upload",
            show_cancel: true,
            confirm_label: Some("Open"),
            extensions: crate::store::MODEL_EXTENSIONS,
        }
    }
}

/// The hit keys the EXPLORER publishes, documented beside the publisher
/// (`automation::hit_keys`) rather than in each host panel: the widget set is
/// the explorer's, only the `hit_prefix` is the host's, so one registration per
/// prefix covers every key the shell can draw. A file ROW is keyed by its own
/// displayed name (`open:bracket.nbrep`), which is what lets a script open
/// a document through the modal the way a user does.
///
/// Until a script opened the modal these keys had never been published under
/// `hit_keys_check`: every native script reached its document through
/// `session_start`'s `document` or `document_open`, both of which bypass the
/// interface entirely.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "file", prefix: "open:", meaning: "the Open modal's explorer: open:<file name> (a row), open:confirm, open:filter, open:back / forward / up, open:path-edit, open:path-edit-toggle, open:crumb:<i>, open:place:<label>, open:pin:<label>, open:unpin:<label>, open:pin-current, open:new-folder, open:new-folder-name, open:hidden-toggle, open:sort:<column>", command: Some("doc_load") },
    HitKeyDoc { panel: "file", prefix: "import:", meaning: "the Import modal's explorer, same key set under the `import` prefix, plus import:upload", command: Some("doc_import") },
    HitKeyDoc { panel: "file", prefix: "saveas:", meaning: "the Save As modal's destination explorer (saveas:file:<key>) and saveas:upload", command: Some("doc_json") },
    HitKeyDoc { panel: "file", prefix: "insert:", meaning: "the Insert-component modal: insert:lib:<name> for a parts-library entry, insert:model:<key> for the store explorer, insert:upload", command: None },
    HitKeyDoc { panel: "file", prefix: "del:", meaning: "delete the file on that explorer row (del:<file name>)", command: None },
    HitKeyDoc { panel: "file", prefix: "upload", meaning: "the explorer's Upload… action where the platform has real-file interchange", command: None },
    HitKeyDoc { panel: "stepparts", prefix: "steplib:dest:", meaning: "the parts-library destination explorer, the same key set under the `steplib:dest` prefix", command: None },
];

#[derive(Default)]
pub struct FileExplorerOutput {
    /// A file was CONFIRMED (double-click, Enter, or the footer confirm button):
    /// the caller acts on it and closes the modal.
    pub activated: Option<String>,
    /// The file currently highlighted, persisting across frames until the
    /// browser location changes. Mirrors the live selection for callers that
    /// want it (and for the headed verifier). `None` when a directory or nothing
    /// is selected.
    pub selected: Option<String>,
    /// A file row was single-clicked THIS frame — a one-shot event. Name-field
    /// callers (Save As, step-parts) fill their field from it without clobbering
    /// keystrokes the user makes afterwards.
    pub picked: Option<String>,
    pub remove: Option<String>,
    pub import: bool,
    pub cancel: bool,
    pub hits: Vec<(String, egui::Rect)>,
}

/// Which column the file table is sorted by.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum SortKey {
    #[default]
    Name,
    Kind,
    Size,
    Date,
}

/// Embeddable browser state. The search text deliberately survives modal uses,
/// making repeated Open / ACOMP operations retain the user's working filter.
#[derive(Default)]
pub struct FileExplorer {
    query: String,
    new_folder: String,
    error: String,
    /// The highlighted FILE identity (directories are navigated, never selected).
    /// Persists across frames so the footer confirm button and keyboard Enter
    /// have a stable target; cleared when the browser location changes so a
    /// stale highlight never leaks into a different directory.
    selected: Option<String>,
    /// The location `selected` belongs to — a mismatch means we walked into
    /// another directory and must drop the highlight.
    selected_at: String,
    /// Visited-location back / forward stacks for the nav buttons.
    history: Vec<String>,
    forward: Vec<String>,
    /// `Some` while the breadcrumb is swapped for an editable path field.
    editing_path: Option<String>,
    /// Grab focus for the path field on the frame it appears.
    path_focus: bool,
    /// The visible FILE identities in display order — the keyboard-nav target
    /// set, refreshed every frame by [`Self::show_list`].
    nav_files: Vec<String>,
    /// Which column the file table sorts by, and its direction (`false` = asc).
    sort_key: SortKey,
    sort_desc: bool,
    /// Show dotfile / hidden entries in the list. Off by default; toggling it on
    /// never affects navigation — a hidden path can always be entered directly.
    show_hidden: bool,
}

impl FileExplorer {
    pub fn new() -> Self {
        Self::default()
    }

    /// The file identity currently highlighted (for the headed verifier and any
    /// caller mirroring the selection). `None` when nothing / a directory holds
    /// the highlight.
    pub fn selected(&self) -> Option<&str> {
        self.selected.as_deref()
    }

    /// Drop the file highlight. The host calls it when the explorer is shown for
    /// another PURPOSE: one explorer serves Import, Insert component and Open in
    /// the same folder, and a file highlighted for one of them is not a choice
    /// made for the next. Without it a `.kicad_sym` picked in Import was still
    /// Insert component's footer choice, and Insert sent it to the parts library.
    pub fn clear_selection(&mut self) {
        self.selected = None;
    }

    /// Browse the model entries exposed by a platform [`ModelStore`].
    pub fn show_store(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        options: FileExplorerOptions<'_>,
    ) -> FileExplorerOutput {
        let entries = store.browser_entries(options.extensions);
        let location = store.browser_location();
        if location != self.selected_at {
            self.selected = None;
            self.selected_at = location.clone();
        }
        let places = store.browser_places();
        let pins = Self::read_pins(store);
        let mut out = FileExplorerOutput::default();

        let selected_before = self.selected.clone();
        self.show_nav_bar(ui, store, &location, options, &mut out);
        if !self.error.is_empty() {
            ui.colored_label(ui.visuals().error_fg_color, &self.error);
        }
        ui.add_space(2.0);
        ui.separator();

        // The explorer FILLS the region its host gave it — a modal's resizable
        // body ([`dialog_body`]), a window's content — rather than carrying a
        // resize handle of its own around the file table: the footer pins to the
        // bottom (under anything the host already pinned there) and the browse
        // area takes every pixel between it and the nav bar, so the file list is
        // the only thing that scrolls.
        bottom_panel(ui, "brep-file-explorer-footer", options.hit_prefix, |ui| {
            self.show_footer(ui, options, &mut out)
        });
        ui.horizontal_top(|ui| {
            if !places.is_empty() {
                ui.vertical(|ui| {
                    self.show_sidebar(ui, store, &places, &pins, &location, options, &mut out);
                });
                ui.separator();
            }
            ui.vertical(|ui| {
                self.show_list(ui, store, &entries, options, &mut out);
            });
        });

        self.handle_keys(ui, options, &mut out);
        out.selected = self.selected.clone();
        // The footer (and any host footer: a name field, a confirm button) was
        // drawn BEFORE the row that just changed the highlight, so it is showing
        // last frame's selection. Ask for one more frame rather than leaving a
        // confirm button greyed out until the pointer happens to move again.
        if out.picked.is_some() || self.selected != selected_before {
            ui.ctx().request_repaint();
        }
        out
    }

    // --- navigation with history ---------------------------------------------

    /// Run a navigation action and, when it actually changes location, record the
    /// previous location on the back stack (clearing the forward stack) and drop
    /// the file highlight. Shared by up / breadcrumb / place / enter-directory.
    fn nav<F: FnOnce() -> Result<(), String>>(&mut self, store: &dyn ModelStore, action: F) {
        let before = store.browser_location();
        match action() {
            Ok(()) => {
                let after = store.browser_location();
                if after != before {
                    self.history.push(before);
                    self.forward.clear();
                }
                self.error.clear();
                self.selected = None;
            }
            Err(error) => self.error = error,
        }
    }

    /// Step back to the previous location, remembering the current one for
    /// forward. A failed navigate restores the stack unchanged.
    fn back(&mut self, store: &dyn ModelStore) {
        if let Some(prev) = self.history.pop() {
            let current = store.browser_location();
            if store.browser_navigate(&prev).is_ok() {
                self.forward.push(current);
                self.selected = None;
                self.error.clear();
            } else {
                self.history.push(prev);
            }
        }
    }

    fn forward_go(&mut self, store: &dyn ModelStore) {
        if let Some(next) = self.forward.pop() {
            let current = store.browser_location();
            if store.browser_navigate(&next).is_ok() {
                self.history.push(current);
                self.selected = None;
                self.error.clear();
            } else {
                self.forward.push(next);
            }
        }
    }

    // --- top navigation bar: back/forward/up + breadcrumb + new folder --------

    fn show_nav_bar(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        location: &str,
        options: FileExplorerOptions<'_>,
        out: &mut FileExplorerOutput,
    ) {
        ui.horizontal(|ui| {
            let back = ui.add_enabled(!self.history.is_empty(), egui::Button::new("\u{25C0}").small());
            out.hits.push((format!("{}:back", options.hit_prefix), back.rect));
            if back.clicked() {
                self.back(store);
            }
            let fwd = {
            let b = crate::icon_text::icon_button(ui, "\u{25B6}").small();
            ui.add_enabled(!self.forward.is_empty(), b)
        };
            out.hits.push((format!("{}:forward", options.hit_prefix), fwd.rect));
            if fwd.clicked() {
                self.forward_go(store);
            }
            let up = ui.small_button("\u{2191}");
            out.hits.push((format!("{}:up", options.hit_prefix), up.rect));
            if up.clicked() {
                self.nav(store, || store.browser_up());
            }
            ui.separator();
            self.show_breadcrumb(ui, store, location, options, out);
        });
        ui.horizontal(|ui| {
            let create = ui.small_button("+ Folder");
            out.hits
                .push((format!("{}:new-folder", options.hit_prefix), create.rect));
            let field = ui.add(
                egui::TextEdit::singleline(&mut self.new_folder)
                    .hint_text("New folder")
                    .desired_width(140.0),
            );
            out.hits
                .push((format!("{}:new-folder-name", options.hit_prefix), field.rect));
            let submit = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if (create.clicked() || submit) && !self.new_folder.trim().is_empty() {
                match store.browser_create_dir(self.new_folder.trim()) {
                    Ok(()) => {
                        self.new_folder.clear();
                        self.error.clear();
                    }
                    Err(error) => self.error = error,
                }
            }
        });
    }

    /// Either a clickable breadcrumb (each ancestor navigates) with a pencil
    /// toggle, or — while editing — a path field that navigates on Enter.
    fn show_breadcrumb(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        location: &str,
        options: FileExplorerOptions<'_>,
        out: &mut FileExplorerOutput,
    ) {
        if self.editing_path.is_some() {
            let mut buf = self.editing_path.take().unwrap();
            let resp = ui.add(
                egui::TextEdit::singleline(&mut buf)
                    .hint_text("type a path, Enter to go")
                    .desired_width(300.0),
            );
            out.hits
                .push((format!("{}:path-edit", options.hit_prefix), resp.rect));
            if self.path_focus {
                resp.request_focus();
                self.path_focus = false;
            }
            let go = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if go {
                self.nav(store, || store.browser_navigate(&buf));
            } else if !resp.lost_focus() {
                self.editing_path = Some(buf); // still editing
            }
            // lost focus without Enter → cancel (editing_path stays None)
            return;
        }
        // The pencil comes FIRST, outside the crumbs' scroll area: at the end of
        // a long path it sat past the dialog's right edge (x = 1910 in a
        // 1400-wide window), where no one could press it.
        let edit = ui.add(crate::icon_text::icon_button(ui, "\u{270E}").small()); // ✎
        out.hits
            .push((format!("{}:path-edit-toggle", options.hit_prefix), edit.rect));
        if edit.on_hover_text("Type a path").clicked() {
            self.editing_path = Some(location.to_string());
            self.path_focus = true;
        }
        let avail = ui.available_width();
        egui::ScrollArea::horizontal()
            .id_salt(format!("{}-crumbs", options.hit_prefix))
            .max_width(avail)
            // Fill the row width (so a long path scrolls horizontally) but shrink
            // to ONE row's height — `false` here would balloon vertically to the
            // modal's huge available height and push the dialog off-screen.
            .auto_shrink([false, true])
            // The folder you are IN is the crumb that matters: a long path shows
            // its tail, and scrolls back to its root.
            .stick_to_right(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    for (i, (label, path)) in breadcrumb_segments(location).into_iter().enumerate() {
                        if i > 0 {
                            ui.weak("\u{203A}"); // ›
                        }
                        let seg = ui.add(egui::Button::new(label).frame(false).small());
                        out.hits
                            .push((format!("{}:crumb:{i}", options.hit_prefix), seg.rect));
                        if seg.clicked() {
                            self.nav(store, || store.browser_navigate(&path));
                        }
                    }
                });
            });
    }

    // --- left sidebar: places + pinned folders --------------------------------

    #[allow(clippy::too_many_arguments)]
    fn show_sidebar(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        places: &[BrowserPlace],
        pins: &[String],
        location: &str,
        options: FileExplorerOptions<'_>,
        out: &mut FileExplorerOutput,
    ) {
        ui.set_min_width(132.0);
        ui.set_max_width(152.0);
        egui::ScrollArea::vertical()
            .id_salt(format!("{}-sidebar", options.hit_prefix))
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.weak("Places");
                for place in places {
                    let glyph = place_glyph(place.kind);
                    let resp = ui.selectable_label(false, format!("{glyph} {}", place.label));
                    out.hits
                        .push((format!("{}:place:{}", options.hit_prefix, place.label), resp.rect));
                    if resp.clicked() {
                        let loc = place.location.clone();
                        self.nav(store, || store.browser_navigate(&loc));
                    }
                }
                if !pins.is_empty() {
                    ui.add_space(6.0);
                    ui.weak("Pinned");
                    for pin in pins {
                        ui.horizontal(|ui| {
                            let label = pin.rsplit(['/', '\\']).find(|s| !s.is_empty()).unwrap_or(pin);
                            let resp = ui.selectable_label(false, format!("\u{1F4CC} {label}"));
                            out.hits
                                .push((format!("{}:pin:{label}", options.hit_prefix), resp.rect));
                            if resp.clicked() {
                                let loc = pin.clone();
                                self.nav(store, || store.browser_navigate(&loc));
                            }
                            let x = {
                                let b = crate::icon_text::icon_button(ui, "\u{2715}").small();
                                ui.add(b)
                            };
                            out.hits
                                .push((format!("{}:unpin:{label}", options.hit_prefix), x.rect));
                            if x.clicked() {
                                Self::set_pin(store, pin, false);
                            }
                        });
                    }
                }
                ui.add_space(6.0);
                let pinned_now = pins.iter().any(|p| p == location);
                let label = if pinned_now {
                    "\u{2715} Unpin folder"
                } else {
                    "\u{1F4CC} Pin folder"
                };
                let btn = ui.small_button(label);
                out.hits
                    .push((format!("{}:pin-current", options.hit_prefix), btn.rect));
                if btn.clicked() {
                    Self::set_pin(store, location, !pinned_now);
                }
            });
    }

    /// Read the pinned-locations list (a JSON array under the reserved key).
    fn read_pins(store: &dyn ModelStore) -> Vec<String> {
        store
            .read(PINNED_KEY)
            .and_then(|raw| serde_json::from_str::<Vec<String>>(&raw).ok())
            .unwrap_or_default()
    }

    /// Add or remove `location` from the pinned list, persisting the result.
    fn set_pin(store: &dyn ModelStore, location: &str, want: bool) {
        let mut pins = Self::read_pins(store);
        let has = pins.iter().any(|p| p == location);
        if want && !has {
            pins.push(location.to_string());
        } else if !want && has {
            pins.retain(|p| p != location);
        } else {
            return;
        }
        let _ = store.write(
            PINNED_KEY,
            &serde_json::to_string(&pins).unwrap_or_else(|_| "[]".into()),
        );
    }

    // --- central file list ----------------------------------------------------

    fn show_list(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        entries: &[BrowserEntry],
        options: FileExplorerOptions<'_>,
        out: &mut FileExplorerOutput,
    ) {
        // Header row: hidden toggle (left) + filter (right).
        ui.horizontal(|ui| {
            let hidden = ui.checkbox(&mut self.show_hidden, "Hidden");
            out.hits
                .push((format!("{}:hidden-toggle", options.hit_prefix), hidden.rect));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let search = ui.add(
                    egui::TextEdit::singleline(&mut self.query)
                        .hint_text("Filter files")
                        .desired_width(150.0),
                );
                out.hits
                    .push((format!("{}:filter", options.hit_prefix), search.rect));
            });
        });

        let query = self.query.trim().to_ascii_lowercase();
        let mut visible: Vec<&BrowserEntry> = entries
            .iter()
            .filter(|entry| {
                // Dotfiles hide unless toggled on; the filter box applies to files.
                let hidden = entry.name.starts_with('.');
                (self.show_hidden || !hidden)
                    && (entry.is_dir
                        || query.is_empty()
                        || entry.name.to_ascii_lowercase().contains(&query))
            })
            .collect();
        // Directories always first; within each group, by the active column.
        let (key, desc) = (self.sort_key, self.sort_desc);
        visible.sort_by(|a, b| {
            b.is_dir.cmp(&a.is_dir).then_with(|| {
                let ord = match key {
                    SortKey::Name => cmp_name(a, b),
                    SortKey::Kind => entry_kind(a).cmp(&entry_kind(b)).then_with(|| cmp_name(a, b)),
                    SortKey::Size => a.size.unwrap_or(0).cmp(&b.size.unwrap_or(0)).then_with(|| cmp_name(a, b)),
                    SortKey::Date => cmp_opt(a.modified, b.modified).then_with(|| cmp_name(a, b)),
                };
                if desc { ord.reverse() } else { ord }
            })
        });
        self.nav_files = visible
            .iter()
            .filter(|entry| !entry.is_dir)
            .map(|entry| entry.identity.clone())
            .collect();

        if visible.is_empty() {
            ui.separator();
            ui.weak(options.empty_label);
            return;
        }

        // The PRIMARY layout: a sortable Name / Type / Size / Date table whose
        // body scrolls inside the height the host left us. Header cells toggle
        // the sort.
        let row_h = egui::TextStyle::Body.resolve(ui.style()).size + 6.0;
        let mut clicked_sort: Option<SortKey> = None;
        TableBuilder::new(ui)
            .id_salt(format!("{}-table", options.hit_prefix))
            .striped(true)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::remainder().at_least(150.0).clip(true)) // Name
            .column(Column::auto().at_least(72.0)) // Type
            .column(Column::auto().at_least(66.0)) // Size
            .column(Column::auto().at_least(120.0)) // Date
            .column(Column::auto().at_least(20.0)) // delete
            .header(row_h, |mut header| {
                for (label, col) in [
                    ("Name", SortKey::Name),
                    ("Type", SortKey::Kind),
                    ("Size", SortKey::Size),
                    ("Date", SortKey::Date),
                ] {
                    header.col(|ui| {
                        let arrow = if key == col {
                            if desc {
                                " \u{25BE}"
                            } else {
                                " \u{25B4}"
                            }
                        } else {
                            ""
                        };
                        let btn =
                            ui.add(egui::Button::new(format!("{label}{arrow}")).frame(false));
                        out.hits.push((
                            format!("{}:sort:{}", options.hit_prefix, sort_slug(col)),
                            btn.rect,
                        ));
                        if btn.clicked() {
                            clicked_sort = Some(col);
                        }
                    });
                }
                header.col(|_ui| {});
            })
            .body(|body| {
                body.rows(row_h, visible.len(), |mut row| {
                    let entry = visible[row.index()];
                    let icon = if entry.is_dir {
                        "\u{1F5C0}"
                    } else {
                        options.row_icon
                    };
                    // An explicit selection wins; before the user picks a row, the
                    // caller's `current` document shows highlighted.
                    let highlight = match &self.selected {
                        Some(sel) => !entry.is_dir && sel == &entry.identity,
                        None => options.current == Some(entry.identity.as_str()),
                    };
                    row.col(|ui| {
                        let r = crate::icon_text::selectable_icon_label(
                            ui,
                            highlight,
                            &format!("{icon} {}", entry.name),
                        );
                        out.hits
                            .push((format!("{}:{}", options.hit_prefix, entry.name), r.rect));
                        if r.double_clicked() {
                            if entry.is_dir {
                                self.nav(store, || store.browser_enter(&entry.identity));
                            } else {
                                self.selected = Some(entry.identity.clone());
                                out.activated = Some(entry.identity.clone());
                            }
                        } else if r.clicked() {
                            if entry.is_dir {
                                self.nav(store, || store.browser_enter(&entry.identity));
                            } else {
                                self.selected = Some(entry.identity.clone());
                                out.picked = Some(entry.identity.clone());
                            }
                        }
                    });
                    row.col(|ui| {
                        ui.weak(entry_kind(entry));
                    });
                    row.col(|ui| {
                        ui.weak(fmt_size(entry.size));
                    });
                    row.col(|ui| {
                        ui.weak(fmt_date(entry.modified));
                    });
                    row.col(|ui| {
                        if options.allow_delete && !entry.is_dir {
                            let del = {
                                let b = crate::icon_text::icon_button(ui, "\u{2715}").small();
                                ui.add(b)
                            };
                            out.hits.push((format!("del:{}", entry.name), del.rect));
                            if del.clicked() {
                                out.remove = Some(entry.identity.clone());
                            }
                        }
                    });
                });
            });
        if let Some(col) = clicked_sort {
            if self.sort_key == col {
                self.sort_desc = !self.sort_desc;
            } else {
                self.sort_key = col;
                self.sort_desc = false;
            }
        }
    }

    /// Keyboard navigation over the visible FILES: ↑/↓ move the selection, Enter
    /// confirms it. Only active when the caller offers a confirm action and no
    /// text field (filter / name / path) holds focus, so typing is never hijacked.
    fn handle_keys(
        &mut self,
        ui: &egui::Ui,
        options: FileExplorerOptions<'_>,
        out: &mut FileExplorerOutput,
    ) {
        if options.confirm_label.is_none() || ui.ctx().egui_wants_keyboard_input() {
            return;
        }
        if self.nav_files.is_empty() {
            return;
        }
        let (down, up, enter) = ui.input(|i| {
            (
                i.key_pressed(egui::Key::ArrowDown),
                i.key_pressed(egui::Key::ArrowUp),
                i.key_pressed(egui::Key::Enter),
            )
        });
        if down || up {
            let current = self
                .selected
                .as_deref()
                .and_then(|s| self.nav_files.iter().position(|f| f == s));
            let next = match current {
                Some(i) if down => (i + 1).min(self.nav_files.len() - 1),
                Some(i) => i.saturating_sub(1),
                None => 0,
            };
            self.selected = Some(self.nav_files[next].clone());
        }
        if enter {
            if let Some(sel) = &self.selected {
                out.activated = Some(sel.clone());
            }
        }
    }

    fn show_footer(
        &self,
        ui: &mut egui::Ui,
        options: FileExplorerOptions<'_>,
        out: &mut FileExplorerOutput,
    ) {
        ui.separator();
        ui.horizontal(|ui| {
            if let Some(label) = options.confirm_label {
                let confirm =
                    ui.add_enabled(self.selected.is_some(), egui::Button::new(label));
                out.hits
                    .push((format!("{}:confirm", options.hit_prefix), confirm.rect));
                if confirm.clicked() {
                    out.activated = self.selected.clone();
                }
            }
            if options.allow_import {
                let import = {
                let label = format!("\u{2B06} {}", options.import_label);
                let b = crate::icon_text::icon_button(ui, &label);
                ui.add(b)
            };
                out.hits.push((options.import_hit.into(), import.rect));
                out.import = import.clicked();
            }
            if options.show_cancel {
                let cancel = ui.button("Cancel");
                out.hits.push(("cancel".into(), cancel.rect));
                out.cancel = cancel.clicked();
            }
            // Selection preview (bare file name), right-aligned.
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if let Some(sel) = &self.selected {
                    let name = sel.rsplit(['/', '\\']).next().unwrap_or(sel);
                    // Truncated: the preview is the LAST thing on the footer row,
                    // so a long file name laid out at its natural width runs left
                    // across the Import / Cancel buttons it is meant to sit beside.
                    ui.add(
                        egui::Label::new(egui::RichText::new(name).weak()).truncate(),
                    );
                }
            });
        });
    }

}

/// Wrap a file dialog's WHOLE body — heading, browser, footer and all — in the
/// shared resizable frame. The drag grip therefore lands at the dialog WINDOW's
/// bottom-right corner instead of around the file table, and dragging it grows
/// the dialog itself in both axes, capped to the viewport so it can never grow
/// off-screen. One shared id means resizing any file dialog resizes them all.
///
/// Only for MODAL hosts: an [`egui::Window`] already carries its own corner
/// grip, and a second frame inside it would put back exactly the handle this
/// removes.
pub fn dialog_body<R>(ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let screen = ui.ctx().content_rect().size();
    let max = egui::vec2((screen.x - 40.0).max(320.0), (screen.y - 80.0).max(240.0));
    egui::Resize::default()
        .id_salt("brep-file-dialog-size")
        // The modal frame already draws the dialog's border; the resize region
        // fills it exactly, so its own stroke would only double it up.
        .with_stroke(false)
        .min_size(egui::vec2(460.0_f32.min(max.x), 300.0_f32.min(max.y)))
        .max_size(max)
        .default_size(egui::vec2(620.0_f32.min(max.x), 520.0_f32.min(max.y)))
        .show(ui, add_contents)
}

/// Pin a dialog's own trailing widgets — a status line, a name field, the
/// action buttons — to the BOTTOM of the dialog. Call it BEFORE
/// [`FileExplorer::show_store`]: the explorer then fills the gap that is left,
/// which is what keeps the buttons still while the file list scrolls. `salt`
/// distinguishes one dialog's footer from another's (two can be on screen at
/// once), and the widgets inside are drawn top-down in the order written.
pub fn dialog_footer<R>(
    ui: &mut egui::Ui,
    salt: &str,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    bottom_panel(ui, "brep-file-dialog-footer", salt, add_contents)
}

/// The one bottom-pinned strip both footers use: no frame and no separator line
/// of its own, so it reads as part of the dialog rather than as a panel, and it
/// takes exactly the height its contents need.
fn bottom_panel<R>(
    ui: &mut egui::Ui,
    base: &str,
    salt: &str,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    egui::containers::panel::Panel::bottom(egui::Id::new((base, salt)))
        .frame(egui::Frame::NONE)
        .show_separator_line(false)
        .show(ui, add_contents)
        .inner
}

/// Case-insensitive name order — the tie-breaker for every column sort.
fn cmp_name(a: &BrowserEntry, b: &BrowserEntry) -> Ordering {
    a.name.to_ascii_lowercase().cmp(&b.name.to_ascii_lowercase())
}

/// Order two optional timestamps, sorting `None` (unknown, e.g. web) last.
fn cmp_opt(a: Option<f64>, b: Option<f64>) -> Ordering {
    match (a, b) {
        (Some(x), Some(y)) => x.partial_cmp(&y).unwrap_or(Ordering::Equal),
        (Some(_), None) => Ordering::Less,
        (None, Some(_)) => Ordering::Greater,
        (None, None) => Ordering::Equal,
    }
}

/// The Type column text for an entry (Folder / BREP model / extension / File).
fn entry_kind(entry: &BrowserEntry) -> String {
    if entry.is_dir {
        return "Folder".into();
    }
    let lower = entry.name.to_ascii_lowercase();
    if lower.ends_with(".nbrep") {
        return "BREP model".into();
    }
    match entry.name.rsplit_once('.') {
        Some((stem, ext)) if !stem.is_empty() && !ext.is_empty() => ext.to_uppercase(),
        _ => "File".into(),
    }
}

/// Stable slug for a sort column's hit key.
fn sort_slug(key: SortKey) -> &'static str {
    match key {
        SortKey::Name => "name",
        SortKey::Kind => "type",
        SortKey::Size => "size",
        SortKey::Date => "date",
    }
}

/// Human-readable byte size, or an em-dash when the backend cannot report it.
fn fmt_size(size: Option<u64>) -> String {
    match size {
        None => "\u{2014}".into(),
        Some(bytes) => {
            let b = bytes as f64;
            if bytes < 1024 {
                format!("{bytes} B")
            } else if b < 1024.0 * 1024.0 {
                format!("{:.1} KB", b / 1024.0)
            } else if b < 1024.0 * 1024.0 * 1024.0 {
                format!("{:.1} MB", b / (1024.0 * 1024.0))
            } else {
                format!("{:.1} GB", b / (1024.0 * 1024.0 * 1024.0))
            }
        }
    }
}

/// `YYYY-MM-DD HH:MM` (UTC) from whole Unix seconds, or an em-dash when unknown.
fn fmt_date(modified: Option<f64>) -> String {
    let Some(secs) = modified else {
        return "\u{2014}".into();
    };
    let days = (secs / 86_400.0).floor() as i64;
    let (y, m, d) = civil_from_days(days);
    let sod = ((secs as i64) % 86_400 + 86_400) % 86_400;
    format!("{y:04}-{m:02}-{d:02} {:02}:{:02}", sod / 3600, (sod % 3600) / 60)
}

/// Gregorian `(year, month, day)` from a day count since the Unix epoch
/// (Howard Hinnant's `civil_from_days`) — pure integer arithmetic, so it is
/// wasm-safe and never touches `SystemTime`.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = (if mp < 10 { mp + 3 } else { mp - 9 }) as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// Split a POSIX-ish location into `(label, navigable-path)` breadcrumb segments,
/// starting at the root. e.g. `/home/user/models` →
/// `[("/", "/"), ("home", "/home"), ("user", "/home/user"), ("models", "/home/user/models")]`.
fn breadcrumb_segments(location: &str) -> Vec<(String, String)> {
    let mut out = vec![("/".to_string(), "/".to_string())];
    let mut acc = String::new();
    for comp in location.split('/').filter(|s| !s.is_empty()) {
        acc.push('/');
        acc.push_str(comp);
        out.push((comp.to_string(), acc.clone()));
    }
    out
}

/// The built-in glyph a sidebar place draws for its category.
fn place_glyph(kind: PlaceKind) -> &'static str {
    match kind {
        PlaceKind::Home => "\u{2302}",       // ⌂
        PlaceKind::Documents => "\u{1F5CE}", // 🗎
        PlaceKind::Downloads => "\u{2B07}",  // ⬇
        PlaceKind::Models => "\u{1F5C0}",    // 🗀
        PlaceKind::Root => "/",
    }
}

