//! step.parts online model library browser (Assembly workbench).
//!
//! The Assembly toolbar's library button opens THIS ctx-level window. It queries
//! the public step.parts v1 API (`https://api.step.parts/v1`), lists matching
//! standard mechanical parts WITH thumbnails, and — on pick — downloads the
//! part's STEP file, creates a NEW part document from it (an `IMPORT3D` feature
//! carrying the raw STEP text — the app's established STEP-import shape), writes
//! that document to a user-chosen location through the [`ModelStore`] seam
//! (keeping the STEP file's original filename), and adds it to the current
//! assembly as an `ACOMP` component via the ONE insert flow
//! ([`EngineState::insert_component`]).
//!
//! ## Networking
//! `ehttp` abstracts the two targets: native uses a background HTTP thread, wasm
//! uses the browser `fetch`. Every step.parts host (the API, the GitHub-LFS STEP
//! media host, and the Vercel-Blob PNG host) sends `Access-Control-Allow-Origin:
//! *`, so the wasm/browser path is NOT CORS-blocked. Async results marshal back
//! into the synchronous egui frame through `std::sync::mpsc` channels drained at
//! the top of [`StepPartsPanel::show`], each fetch calling `ctx.request_repaint`
//! so the UI wakes when a reply lands. A failed request never panics/hangs — it
//! surfaces as a status line and the dialog stays usable.
//!
//! ## Testability
//! The request/response SHAPE lives in free functions ([`search_url`],
//! [`parse_search_response`], [`step_filename_stem`], [`build_part_document`])
//! and the store-write + component-add lives in [`StepPartsPanel::import_step_text`],
//! all unit-tested with injected data — no network in the test suite.

use crate::http::{fetch_bytes, fetch_text};
use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::parts_library::document_signature;
use crate::panels::file_explorer::{self, FileExplorer, FileExplorerOptions};
use crate::store::{model_display_name, ModelStore};
use brep_render::engine_state::{ComponentInsert, EngineState};
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;
use std::sync::mpsc::Receiver;

/// The public step.parts v1 API base (the app uses the same `/v1` routes an
/// agent would; see `GET /v1/openapi.json`).
const API_BASE: &str = "https://api.step.parts/v1";

/// The feature names [`crate::offsite`]'s refusal sentence starts with.
const SEARCH: &str = "STEP parts search";
const DOWNLOAD: &str = "STEP part download";
/// Results per page. Kept modest (vs. the API's 500 cap) so a search fires a
/// bounded burst of thumbnail fetches; Prev/Next page through the rest.
const PAGE_SIZE: u32 = 24;

/// One search result — the subset of the API `AgentPart` record this UI needs.
/// Field names come straight from `openapi.json` (`stepUrl` = canonical STEP
/// download, `pngUrl` = thumbnail), never guessed.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PartItem {
    pub id: String,
    pub name: String,
    pub description: String,
    pub category: String,
    pub step_url: String,
    pub png_url: String,
}

// --- pure helpers (unit-tested; no egui, no network) -------------------------

/// Percent-encode a query VALUE (RFC 3986 unreserved set kept; everything else,
/// space included, becomes `%XX`). One field only — not worth a urlencoding crate.
fn encode_query(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char)
            }
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

/// The `/v1/parts` search URL for `query` (blank ⇒ plain list) at 1-based `page`.
fn search_url(base: &str, query: &str, page: u32) -> String {
    let mut url = format!("{base}/parts?pageSize={PAGE_SIZE}&page={page}");
    let q = query.trim();
    if !q.is_empty() {
        url.push_str("&q=");
        url.push_str(&encode_query(q));
    }
    url
}

/// A string field off a JSON object, or `""` when absent/non-string.
fn str_field(obj: &Value, key: &str) -> String {
    obj.get(key).and_then(Value::as_str).unwrap_or("").to_string()
}

/// The response's `hasNextPage` flag (defaults `false` when absent).
fn response_has_next(json: &Value) -> bool {
    json.get("hasNextPage").and_then(Value::as_bool).unwrap_or(false)
}

/// Parse a `/v1/parts` response body into result items. Tolerant of missing
/// optional fields; errors only on a structurally wrong body (no `items` array).
fn parse_search_response(json: &str) -> Result<(Vec<PartItem>, bool), String> {
    let v: Value = serde_json::from_str(json).map_err(|e| format!("bad response: {e}"))?;
    // The API also returns `{ "error": "…" }` on a bad request — surface it.
    if let Some(err) = v.get("error").and_then(Value::as_str) {
        return Err(err.to_string());
    }
    let items = v
        .get("items")
        .and_then(Value::as_array)
        .ok_or("response missing `items` array")?;
    let parts = items
        .iter()
        .map(|it| PartItem {
            id: str_field(it, "id"),
            name: str_field(it, "name"),
            description: str_field(it, "description"),
            category: str_field(it, "category"),
            step_url: str_field(it, "stepUrl"),
            png_url: str_field(it, "pngUrl"),
        })
        .collect();
    Ok((parts, response_has_next(&v)))
}

/// The STEP file's ORIGINAL filename stem — the last path segment of `stepUrl`
/// with any query string and a `.step`/`.stp` extension stripped. This becomes
/// the default part-document name so the saved part keeps its source filename.
fn step_filename_stem(step_url: &str) -> String {
    let last = step_url
        .rsplit('/')
        .next()
        .unwrap_or(step_url);
    let last = last.split(['?', '#']).next().unwrap_or(last);
    let stem = last
        .strip_suffix(".step")
        .or_else(|| last.strip_suffix(".STEP"))
        .or_else(|| last.strip_suffix(".stp"))
        .or_else(|| last.strip_suffix(".STP"))
        .unwrap_or(last);
    if stem.is_empty() {
        "imported-part".to_string()
    } else {
        stem.to_string()
    }
}

/// Build a part HistoryRequest document that embeds `step_text` as an `IMPORT3D`
/// feature — the exact shape [`EngineState::import_step_feature`] uses. The
/// ISO-10303-21 gate is enforced HERE, strictly BEFORE any store write or
/// component insert, so a non-STEP payload is refused up front and the
/// insert/library error path (which aborts the NATIVE app — see project memory
/// `jsvalue-native-abort-trap`) is never reached on a bad download.
fn build_part_document(step_text: &str) -> Result<String, String> {
    if !step_text.contains("ISO-10303-21") {
        return Err("not a STEP file (missing the ISO-10303-21 header)".into());
    }
    Ok(serde_json::json!({
        "expressions": "",
        "configurator": {},
        "features": [{
            "type": "IMPORT3D",
            "inputParams": { "id": "IMPORT3D1", "stepText": step_text },
            "persistentData": {}
        }]
    })
    .to_string())
}


/// Decode PNG bytes into an egui texture (thumbnail). `None` on a decode error
/// so the row shows a neutral placeholder rather than failing the whole dialog.
fn decode_thumbnail(ctx: &egui::Context, id: &str, bytes: &[u8]) -> Option<egui::TextureHandle> {
    let image = image::load_from_memory(bytes).ok()?.to_rgba8();
    let (w, h) = image.dimensions();
    let color = egui::ColorImage::from_rgba_unmultiplied([w as usize, h as usize], image.as_raw());
    Some(ctx.load_texture(format!("steplib-thumb:{id}"), color, egui::TextureOptions::LINEAR))
}

/// A thumbnail's lifecycle for one result row.
enum ThumbState {
    Loading,
    Failed,
    Ready(egui::TextureHandle),
}

/// Which sub-view the window is showing.
#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    /// Search box + results list.
    Search,
    /// Destination picker for the part being imported.
    Destination,
}

/// The Assembly workbench's step.parts library browser (shell-owned, ctx-level
/// window — the Info/Interference idiom). Opened by the toolbar button; owns all
/// of its own search / thumbnail / import state.
pub struct StepPartsPanel {
    open: bool,
    view: View,
    /// The search field text.
    query: String,
    /// 1-based page currently shown.
    page: u32,
    /// Whether the API reported a further page (drives Next).
    has_next: bool,
    /// A search request is in flight.
    searching: bool,
    /// Human status / error line (search + import both write here).
    status: String,
    /// Current page of results.
    results: Vec<PartItem>,
    /// In-flight search reply channel.
    search_rx: Option<Receiver<Result<String, String>>>,
    /// Per-result thumbnail state, keyed by part id.
    thumbs: HashMap<String, ThumbState>,
    /// In-flight thumbnail byte channels, keyed by part id.
    thumb_rx: HashMap<String, Receiver<Result<Vec<u8>, String>>>,
    /// The part chosen for import (drives the Destination view).
    pending: Option<PartItem>,
    /// Destination document name — prefilled with the STEP filename stem.
    dest_name: String,
    /// The downloaded STEP text for `pending`, once it lands.
    pending_step: Option<String>,
    /// In-flight STEP download channel.
    step_rx: Option<Receiver<Result<String, String>>>,
    /// Embeddable store browser for choosing the save LOCATION.
    explorer: FileExplorer,
    /// Whether the first (blank-query) search has been kicked since opening.
    seeded: bool,
    /// Per-frame widget rects for the headed verifier.
    hits: HashMap<String, egui::Rect>,
}

impl Default for StepPartsPanel {
    fn default() -> Self {
        Self {
            open: false,
            view: View::Search,
            query: String::new(),
            page: 1,
            has_next: false,
            searching: false,
            status: String::new(),
            results: Vec::new(),
            search_rx: None,
            thumbs: HashMap::new(),
            thumb_rx: HashMap::new(),
            pending: None,
            dest_name: String::new(),
            pending_step: None,
            step_rx: None,
            explorer: FileExplorer::new(),
            seeded: false,
            hits: HashMap::new(),
        }
    }
}

impl StepPartsPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Toolbar entry point: open the window (a blank-query first page is fetched
    /// on the first frame so the list is never empty on open).
    pub fn open(&mut self) {
        self.open = true;
        self.view = View::Search;
        self.seeded = false;
    }

    /// The network work in flight, for the status bar's working indicator
    /// ([`super::busy`]): a STEP download outranks a search. Thumbnails are
    /// not reported — they fill in beside results already on screen.
    pub fn busy_activities(&self) -> Vec<super::busy::Activity> {
        let mut out = Vec::new();
        if self.step_rx.is_some() {
            out.push(super::busy::Activity::new(
                "stepParts:download",
                "download",
                format!("Downloading {}", self.dest_name),
            ));
        }
        if self.search_rx.is_some() {
            out.push(super::busy::Activity::new("stepParts:search", "search", "Searching STEP parts"));
        }
        out
    }

    fn hit(&mut self, key: &str, resp: &egui::Response) {
        self.hits.insert(key.to_string(), resp.rect);
    }

    // --- search ---------------------------------------------------------------

    /// Kick a search for the current `query` at `page` (clears the previous
    /// page's results + thumbnails so no stale texture leaks across a query).
    fn start_search(&mut self, ctx: &egui::Context, page: u32) {
        self.page = page.max(1);
        self.searching = true;
        self.status = "searching…".into();
        self.results.clear();
        self.thumbs.clear();
        self.thumb_rx.clear();
        let url = search_url(API_BASE, &self.query, self.page);
        // On a PLM-hosted app that does not allow the API, say so instead of
        // letting the browser refuse the request (crate::offsite).
        if let Some(sentence) = crate::offsite::refusal(SEARCH, &url) {
            self.searching = false;
            self.status = sentence;
            return;
        }
        self.search_rx = Some(fetch_text(ctx, url));
    }

    /// Populate results from a raw search response body, then kick a thumbnail
    /// fetch per result. Split out so a test can inject a stub response.
    fn apply_search_response(&mut self, ctx: &egui::Context, body: &str) {
        match parse_search_response(body) {
            Ok((parts, has_next)) => {
                self.has_next = has_next;
                self.status = if parts.is_empty() {
                    "no matches".into()
                } else {
                    format!("{} result{}", parts.len(), if parts.len() == 1 { "" } else { "s" })
                };
                for part in &parts {
                    // A thumbnail host the server does not allow is a missing
                    // thumbnail, not a status line per result.
                    if !part.png_url.is_empty() && crate::offsite::refusal(SEARCH, &part.png_url).is_some() {
                        self.thumbs.insert(part.id.clone(), ThumbState::Failed);
                    } else if !part.png_url.is_empty() {
                        self.thumbs.insert(part.id.clone(), ThumbState::Loading);
                        self.thumb_rx
                            .insert(part.id.clone(), fetch_bytes(ctx, part.png_url.clone()));
                    }
                }
                self.results = parts;
            }
            Err(err) => {
                self.status = format!("search failed: {err}");
                self.results.clear();
                self.has_next = false;
            }
        }
    }

    /// Drain any in-flight async replies (search, thumbnails, STEP download).
    fn poll(&mut self, ctx: &egui::Context) {
        if let Some(rx) = &self.search_rx {
            if let Ok(reply) = rx.try_recv() {
                self.search_rx = None;
                self.searching = false;
                match reply {
                    Ok(body) => self.apply_search_response(ctx, &body),
                    Err(err) => {
                        let url = search_url(API_BASE, &self.query, self.page);
                        self.status = crate::offsite::explain(SEARCH, &url, format!("search failed: {err}"));
                    }
                }
            }
        }
        // Thumbnails: collect ready/failed ids, then apply (avoid borrow overlap).
        let ready: Vec<(String, Result<Vec<u8>, String>)> = self
            .thumb_rx
            .iter()
            .filter_map(|(id, rx)| rx.try_recv().ok().map(|r| (id.clone(), r)))
            .collect();
        for (id, reply) in ready {
            self.thumb_rx.remove(&id);
            let state = match reply {
                Ok(bytes) => decode_thumbnail(ctx, &id, &bytes)
                    .map(ThumbState::Ready)
                    .unwrap_or(ThumbState::Failed),
                Err(_) => ThumbState::Failed,
            };
            self.thumbs.insert(id, state);
        }
        if let Some(rx) = &self.step_rx {
            if let Ok(reply) = rx.try_recv() {
                self.step_rx = None;
                match reply {
                    Ok(text) => {
                        if text.contains("ISO-10303-21") {
                            self.pending_step = Some(text);
                            self.status = "STEP downloaded — choose a location and save".into();
                        } else {
                            self.status = "download is not a STEP file".into();
                        }
                    }
                    Err(err) => {
                        let url = self.pending.as_ref().map(|p| p.step_url.clone()).unwrap_or_default();
                        self.status = crate::offsite::explain(DOWNLOAD, &url, format!("STEP download failed: {err}"));
                    }
                }
            }
        }
    }

    // --- import ---------------------------------------------------------------

    /// Begin importing `part`: switch to the Destination view, prefill the name
    /// with the STEP stem, and start downloading its STEP file.
    fn begin_import(&mut self, ctx: &egui::Context, part: PartItem) {
        if let Some(sentence) = crate::offsite::refusal(DOWNLOAD, &part.step_url) {
            self.status = sentence;
            return;
        }
        self.dest_name = step_filename_stem(&part.step_url);
        self.pending_step = None;
        self.status = format!("downloading {}…", self.dest_name);
        self.step_rx = Some(fetch_text(ctx, part.step_url.clone()));
        self.pending = Some(part);
        self.view = View::Destination;
    }

    /// Create the part document from `step_text`, write it to the store under
    /// `dest_name` at the current browser location (keeping the original
    /// filename), and add it to the assembly as an `ACOMP`. Returns the new
    /// feature id on success. The store WRITE happens first and its error is
    /// surfaced BEFORE any insert, per the advisor's ordering.
    ///
    /// Free of egui/network so the full write+insert path is unit-testable with
    /// an injected `step_text` + an in-memory store.
    fn import_step_text(
        state: &mut EngineState,
        store: &dyn ModelStore,
        dest_name: &str,
        step_text: &str,
    ) -> Result<String, String> {
        let document = build_part_document(step_text)?;
        // Write the part document to the chosen location first (the realistic
        // failure — e.g. a storage quota on a multi-MB STEP — surfaces here,
        // before we touch the assembly).
        let identity = store.browser_write(dest_name, &document)?;
        let display = model_display_name(&identity);
        let id = state
            .insert_component(ComponentInsert::New {
                name: &display,
                source_key: &identity,
                source_signature: &document_signature(&document),
                document_json: &document,
            })
            .map_err(|e| format!("add component failed: {e}"))?;
        Ok(id)
    }

    // --- rendering ------------------------------------------------------------

    /// Draw the window (if open) at ctx level. `store` is the destination for the
    /// saved part document.
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState, store: &dyn ModelStore) {
        self.hits.clear();
        if !self.open {
            return;
        }
        self.poll(ctx);
        // Seed a first (blank-query) page so the dialog opens with content.
        if !self.seeded && !self.searching && self.results.is_empty() {
            self.seeded = true;
            self.start_search(ctx, 1);
        }

        let mut open = true;
        egui::Window::new("step.parts library")
            .id(egui::Id::new("brep-step-parts-window"))
            .open(&mut open)
            .movable(true)
            .resizable(true)
            .default_size([460.0, 520.0])
            .default_pos([820.0, 70.0])
            .show(ctx, |ui| match self.view {
                View::Search => self.search_view(ui, ctx),
                View::Destination => self.destination_view(ui, state, store),
            });
        self.open = open;
    }

    fn search_view(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.horizontal(|ui| {
            let field = ui.add(
                egui::TextEdit::singleline(&mut self.query)
                    .hint_text("Search parts (e.g. M3 screw, ISO 4762)…")
                    .desired_width(260.0),
            );
            self.hit("steplib:query", &field);
            let enter = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            let search = ui.button("Search");
            self.hit("steplib:search", &search);
            if search.clicked() || enter {
                self.start_search(ctx, 1);
            }
        });
        ui.horizontal(|ui| {
            ui.weak(&self.status);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let next = ui.add_enabled(self.has_next && !self.searching, egui::Button::new("Next ›"));
                self.hit("steplib:next", &next);
                if next.clicked() {
                    self.start_search(ctx, self.page + 1);
                }
                let prev = ui.add_enabled(self.page > 1 && !self.searching, egui::Button::new("‹ Prev"));
                self.hit("steplib:prev", &prev);
                if prev.clicked() {
                    self.start_search(ctx, self.page - 1);
                }
                ui.weak(format!("page {}", self.page));
            });
        });
        ui.separator();

        // Snapshot the ids/urls to act on AFTER the row loop (no borrow overlap
        // with `self.thumbs` reads during the loop).
        let mut chosen: Option<PartItem> = None;
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                if self.results.is_empty() && !self.searching {
                    ui.weak("No results. Try a different search.");
                }
                let results = self.results.clone();
                for (index, part) in results.iter().enumerate() {
                    ui.horizontal(|ui| {
                        // Thumbnail (or placeholder), fixed 56×56 box.
                        let size = egui::vec2(56.0, 56.0);
                        match self.thumbs.get(&part.id) {
                            Some(ThumbState::Ready(tex)) => {
                                ui.add(egui::Image::new(tex).fit_to_exact_size(size));
                            }
                            Some(ThumbState::Loading) => {
                                ui.add_sized(size, egui::Spinner::new());
                            }
                            _ => {
                                let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
                                ui.painter().rect_filled(
                                    rect,
                                    3.0,
                                    ui.visuals().extreme_bg_color,
                                );
                                ui.painter().text(
                                    rect.center(),
                                    egui::Align2::CENTER_CENTER,
                                    "STEP",
                                    egui::TextStyle::Small.resolve(ui.style()),
                                    ui.visuals().weak_text_color(),
                                );
                            }
                        }
                        ui.vertical(|ui| {
                            let row = ui.selectable_label(
                                false,
                                egui::RichText::new(&part.name).strong(),
                            );
                            // Keyed by the row's INDEX on this page, not by the
                            // catalogue id: the id is a live third-party string a
                            // checked-in script cannot name, and an index is what
                            // this panel's registry entry has always documented
                            // (`result:i`, `add:i`). The id is still readable, off
                            // `__brepStepParts.results[i].id`.
                            self.hit(&format!("steplib:result:{index}"), &row);
                            if !part.category.is_empty() {
                                ui.weak(&part.category);
                            }
                            if !part.description.is_empty() {
                                ui.small(truncate(&part.description, 90));
                            }
                            let add = ui.button("Add to assembly");
                            self.hit(&format!("steplib:add:{index}"), &add);
                            if add.clicked() || row.double_clicked() {
                                chosen = Some(part.clone());
                            }
                        });
                    });
                    ui.separator();
                }
            });

        if let Some(part) = chosen {
            self.begin_import(ctx, part);
        }
    }

    fn destination_view(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) {
        let part_name = self
            .pending
            .as_ref()
            .map(|p| p.name.clone())
            .unwrap_or_default();
        ui.heading("Save part & add component");
        ui.weak(&part_name);
        ui.add_space(4.0);

        // The name field, the status line and the actions are this view's
        // footer: pinned to the bottom of the window, so the location picker
        // below fills the rest of it and the window's OWN corner grip stays the
        // only resize handle — the explorer carries none of its own.
        let step_ready = self.pending_step.is_some();
        let mut do_import = false;
        let mut go_back = false;
        file_explorer::dialog_footer(ui, "steplib:dest", |ui| {
            ui.add_space(4.0);
            ui.label("File name (from the STEP file)");
            let field = ui.add(
                egui::TextEdit::singleline(&mut self.dest_name)
                    .hint_text("part name")
                    .desired_width(f32::INFINITY),
            );
            self.hit("steplib:dest-name", &field);

            if !self.status.is_empty() {
                ui.add_space(2.0);
                ui.weak(&self.status);
            }
            ui.add_space(6.0);

            let name_ok = !self.dest_name.trim().is_empty();
            ui.horizontal(|ui| {
                let save = ui.add_enabled(
                    step_ready && name_ok,
                    egui::Button::new("Save & Add"),
                );
                self.hit("steplib:save", &save);
                if save.clicked() {
                    do_import = true;
                }
                let back = ui.button("Back");
                self.hit("steplib:back", &back);
                if back.clicked() {
                    go_back = true;
                }
                if !step_ready {
                    ui.add(egui::Spinner::new());
                    ui.weak("downloading…");
                }
            });
        });

        // Location picker — reuse the shared store explorer (navigation only;
        // the name is fixed to the STEP filename above).
        let options = FileExplorerOptions {
            hit_prefix: "steplib:dest",
            empty_label: "(no saved models here)",
            row_icon: "\u{1F5CE}",
            current: None,
            allow_delete: false,
            allow_import: false,
            import_label: "",
            import_hit: "steplib:dest:upload",
            show_cancel: false,
            confirm_label: None,
            extensions: &["nbrep"],
        };
        let output = self.explorer.show_store(ui, store, options);
        for (key, rect) in output.hits {
            self.hits.insert(key, rect);
        }

        if do_import {
            let step_text = self.pending_step.clone().unwrap_or_default();
            let dest = self.dest_name.trim().to_string();
            match Self::import_step_text(state, store, &dest, &step_text) {
                Ok(id) => {
                    self.status = format!("added {dest} ({id})");
                    self.pending = None;
                    self.pending_step = None;
                    self.view = View::Search;
                }
                Err(err) => self.status = err,
            }
        } else if go_back {
            self.pending = None;
            self.pending_step = None;
            self.step_rx = None;
            self.status.clear();
            self.view = View::Search;
        }
    }

    // --- verifier surface -----------------------------------------------------

    /// The window's logical state for the headed verifier (`__brepStepParts`).
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn state_json(&self) -> String {
        let results: Vec<Value> = self
            .results
            .iter()
            .map(|p| {
                serde_json::json!({
                    "id": p.id,
                    "name": p.name,
                    "category": p.category,
                    "hasThumb": matches!(self.thumbs.get(&p.id), Some(ThumbState::Ready(_))),
                })
            })
            .collect();
        let pending = self.pending.as_ref().map(|p| {
            serde_json::json!({
                "id": p.id,
                "name": p.name,
                "destName": self.dest_name,
                "stepReady": self.pending_step.is_some(),
            })
        });
        serde_json::json!({
            "open": self.open,
            "view": match self.view { View::Search => "search", View::Destination => "destination" },
            "query": self.query,
            "status": self.status,
            "searching": self.searching,
            "page": self.page,
            "hasNext": self.has_next,
            "resultCount": self.results.len(),
            "results": results,
            "pending": pending.unwrap_or(Value::Null),
        })
        .to_string()
    }

    /// Per-frame widget rects for the headed verifier (`__brepStepPartsHit`).
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}

/// Clip `s` to `max` chars with an ellipsis (thumbnail-row description).
fn truncate(s: &str, max: usize) -> String {
    if s.chars().count() <= max {
        s.to_string()
    } else {
        let mut out: String = s.chars().take(max).collect();
        out.push('…');
        out
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "stepparts", prefix: "steplib:", meaning: "the STEP parts library controls (search, query, result:i and add:i by the row's index on the current page, prev, next, back, save, dest-name)", command: None },
];
