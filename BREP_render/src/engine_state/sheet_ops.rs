//! Drawing sheets — the engine half: the document's `sheets` block
//! (checkpointed edits, like `pmi_ops`), which sheet the viewport shows, which
//! sheet object's form is open, and the projection cache the sheet viewport
//! and the SVG export both read.
//!
//! Persisted vs engine memory, the PMI split exactly: the block (sheets, paper
//! sizes, placements) lives in the document and rides its undo stack; WHICH
//! sheet is open and WHICH object's form is showing are engine memory — a
//! mode, not model state — so a rerun, a save or an undo never churns on them.
//!
//! Unlike a PMI edit, a sheet edit never re-runs the history: a sheet places
//! what the PMI tail already resolved and adds no geometry of its own, so
//! writing the block is the whole edit. What it DOES invalidate is the
//! projection cache, which is keyed on the scene's applied-run generation AND
//! the document's revision — so a model rebuild, a placement move and a PMI
//! LABEL DRAG all re-project, while a pan or a zoom does not. The label drag
//! is why the key is not the sheets block alone: it writes the `pmi` block and
//! patches the cached report WITHOUT a run, so a cache keyed on the run and
//! the sheet would draw the label where it used to be. The key's fourth part is
//! the TITLE BLOCK's stamp — the document's name and today's date — because
//! neither lives in the document and a sheet that printed yesterday's date
//! after midnight would be wrong about the one thing a date is for.

use super::*;
use crate::sheets::frame::{today_iso, FrameContext};
use crate::sheets::dimension::{SheetAnchor, SheetDimension, ALIGNED, ALIGNMENTS, KINDS};
use crate::sheets::ordinate::{SheetOrdinate, AXES};
use crate::sheets::{DetailCircle, SectionCut, DETAIL_SCALE_FACTOR};
use crate::sheets::project::{project_sheet_with, Deferred, LinesJob, LinesMode, Model, SheetDrawing};
use crate::sheets::{
    PlacedView, Sheet, SheetState, DEFAULT_BORDER_INSET_MM, DEFAULT_SCALE, DEFAULT_SIZE,
    ORTHOGRAPHIC,
};
use serde_json::Value;

/// A projected sheet and the state it was projected from: `(sheet id, applied
/// run, document revision, the scene's exact revision, the landed passes'
/// revision, the title block's stamp)`.
pub(crate) struct SheetCache {
    pub key: (String, u64, u64, u64, u64, u64, String),
    pub drawing: SheetDrawing,
    /// Every placement drew its exact lines: no pass was handed to the runner
    /// and left the placement on its mesh approximation. An EXPORT reads only
    /// a complete drawing.
    pub complete: bool,
    /// An answer landed while this incomplete drawing was being projected, so
    /// a placement it drew by the mesh may already have its exact lines: the
    /// next call re-projects although the key still matches.
    pub reproject: bool,
    /// The drawing as JSON, serialised ONCE per projection. The `__brepSheets`
    /// registry blob carries it and the registry publishes EVERY frame in the
    /// browser build, so a per-frame `to_string` of the whole projection would
    /// be exactly the O(document) per-frame publisher the registry forbids.
    pub drawing_json: String,
}

/// The exact passes a sheet handed to the RUNNER, so the UI thread never runs
/// one (see [`LinesJob`]).
///
/// ONE job is on the runner at a time and the rest wait here, at most one per
/// placement: a scale typed digit by digit, or a camera re-captured twice,
/// asks for the newest pass rather than queueing every one. A job overtaken by
/// a newer one for the same placement while it runs is SUPERSEDED and its
/// answer is dropped when it lands; a cancel or a document switch drops
/// everything ([`Self::forget`]). A runner that cannot take the job, or
/// answers that it could not draw it, leaves that pass to the UI thread —
/// exact and late rather than approximate for ever.
#[derive(Default)]
pub(crate) struct SheetLinesWork {
    next_id: u64,
    in_flight: Option<LinesFlight>,
    parked: Vec<LinesJob>,
    /// Keys the runner could not draw; asked again, they run here.
    refused: std::collections::HashSet<String>,
    /// The runner took no job at all (a runner without the command).
    unsupported: bool,
    /// Bumps when an answer lands — part of the sheet cache's key.
    revision: u64,
}

struct LinesFlight {
    id: u64,
    placement: String,
    key: String,
    superseded: bool,
}

impl SheetLinesWork {
    /// A job is on the runner or waiting for it.
    pub(crate) fn pending(&self) -> bool {
        self.in_flight.is_some() || !self.parked.is_empty()
    }

    /// Drop every job: the runner they were sent to was cancelled, reset for
    /// another document or replaced, and no answer from it is for this scene.
    pub(crate) fn forget(&mut self) {
        self.in_flight = None;
        self.parked.clear();
        self.refused.clear();
        self.unsupported = false;
        self.revision += 1;
    }

    /// What a projection's exact-lines miss gets: the lines, when the runner
    /// answers at once (an inline runner); pending, while it computes; or the
    /// word to compute them on this thread.
    fn defer(&mut self, runner: &mut dyn crate::runner::HistoryRunner, job: LinesJob) -> Deferred {
        if self.unsupported || self.refused.contains(&job.key) {
            return Deferred::Here;
        }
        let key = job.key.clone();
        match self.in_flight.as_mut() {
            // Asked again for the pass already running: wait for it, and it is
            // wanted after all if a newer job had superseded it.
            Some(flight) if flight.key == key => {
                flight.superseded = false;
                self.parked.retain(|parked| parked.placement != job.placement);
            }
            Some(flight) => {
                if flight.placement == job.placement {
                    flight.superseded = true;
                }
                self.park(job);
            }
            None => self.park(job),
        }
        self.pump(runner);
        if let Some(lines) = crate::sheets::project::cached_exact_lines(&key) {
            return Deferred::Lines(lines);
        }
        if self.unsupported || self.refused.contains(&key) {
            return Deferred::Here;
        }
        Deferred::Pending
    }

    fn park(&mut self, job: LinesJob) {
        self.parked.retain(|parked| parked.placement != job.placement && parked.key != job.key);
        self.parked.push(job);
    }

    /// Land every answer the runner has and hand it the next waiting job.
    pub(crate) fn pump(&mut self, runner: &mut dyn crate::runner::HistoryRunner) {
        loop {
            if self.in_flight.is_none() && !self.parked.is_empty() {
                let job = self.parked.remove(0);
                self.next_id += 1;
                let flight = LinesFlight {
                    id: self.next_id,
                    placement: job.placement.clone(),
                    key: job.key.clone(),
                    superseded: false,
                };
                if runner.submit_sheet_lines(crate::runner::SheetLinesRequest { id: self.next_id, job }) {
                    self.in_flight = Some(flight);
                } else {
                    // Every waiting job runs on the UI thread when it is next
                    // asked for; the cache must re-project to ask.
                    self.unsupported = true;
                    self.parked.clear();
                    self.revision += 1;
                }
            }
            let Some(reply) = runner.poll_sheet_lines() else { break };
            if self.in_flight.as_ref().is_none_or(|flight| flight.id != reply.id) {
                continue; // a runner's answer from before a cancel or a reset
            }
            let flight = self.in_flight.take().expect("checked above");
            if flight.superseded {
                continue;
            }
            match reply.lines {
                Ok(lines) => crate::sheets::project::store_exact_lines(reply.key, lines),
                Err(_) => {
                    self.refused.insert(reply.key);
                }
            }
            self.revision += 1;
        }
    }
}

impl EngineState {
    // --- read surface ------------------------------------------------------

    /// The document's `sheets` block as typed state (the default — no sheets —
    /// when the document carries none).
    pub fn sheet_state(&self) -> SheetState {
        self.history
            .sheets_block()
            .and_then(|block| serde_json::from_value(block.clone()).ok())
            .unwrap_or_default()
    }

    /// The sheet the sheet viewport shows, if any.
    pub fn sheet_open(&self) -> Option<&str> {
        self.sheet_open.as_deref()
    }

    /// The open document's name — what a sheet's TITLE BLOCK prints. The shell
    /// owns it (the document does not carry its own name), so the app sets it
    /// from the active tab; this is a no-op when it has not changed, so it can
    /// be called every frame.
    pub fn set_document_name(&mut self, name: &str) {
        if self.document_name != name {
            self.document_name = name.to_string();
            self.dirty = true;
        }
    }

    pub fn document_name(&self) -> &str {
        &self.document_name
    }

    /// The sheet or placed view whose form the Sheets pane shows.
    pub fn sheet_open_object(&self) -> Option<&str> {
        self.sheet_open_object.as_deref()
    }

    /// How many sheet-object forms this engine has OPENED — the app's dialog
    /// door reads it beside [`Self::sheet_open_object`] so a re-open of the
    /// object already open is still an open (the id alone does not move).
    /// Monotonic and never reset; a CLOSE does not count.
    pub fn sheet_object_opens(&self) -> u64 {
        self.sheet_object_opens
    }

    /// **The sheet half of the dialog door's counter.** Open `id`'s form and
    /// count the open. EVERY path that opens a sheet object's form goes
    /// through here — the pane's row Edit and double click, a click on the
    /// paper, and the place / add-dimension / add-ordinate / new-section /
    /// new-detail adds that open their own result — so
    /// the door has one signal to watch and a re-open of the same object still
    /// reads as an open. Assigning `sheet_open_object = Some(..)` anywhere else
    /// is the bug this exists to prevent.
    ///
    /// An open placement, dimension or ordinate set is the selected one too —
    /// the dialog is about it. (A SHEET's selection is which sheet is open in
    /// the viewport, so opening a sheet's own form selects nothing here.)
    fn open_sheet_object(&mut self, id: String) {
        if Self::is_sheet_child(&self.sheet_state(), &id) {
            self.sheet_selected_object = Some(id.clone());
        }
        self.sheet_open_object = Some(id);
        self.sheet_object_opens = self.sheet_object_opens.wrapping_add(1);
    }

    /// Whether `id` is something a sheet HOLDS — a placement, a dimension or
    /// an ordinate set — rather than a sheet.
    fn is_sheet_child(state: &SheetState, id: &str) -> bool {
        state.locate_view(id).is_some()
            || state.locate_dimension(id).is_some()
            || state.locate_ordinate(id).is_some()
    }

    /// The placement, dimension or ordinate set selected in the Sheets tree
    /// or on the paper, while it exists.
    pub fn sheet_selected_object(&self) -> Option<&str> {
        let id = self.sheet_selected_object.as_deref()?;
        Self::is_sheet_child(&self.sheet_state(), id).then_some(id)
    }

    /// SELECT a placement, dimension or ordinate set without opening its form
    /// — the Sheets tree's single click (`None` deselects). Returns false,
    /// changing nothing, for an id that names none of those.
    pub fn sheet_select_object(&mut self, id: Option<&str>) -> bool {
        match id {
            Some(id) if Self::is_sheet_child(&self.sheet_state(), id) => {
                self.sheet_selected_object = Some(id.to_string());
            }
            Some(_) => return false,
            None => self.sheet_selected_object = None,
        }
        self.dirty = true;
        true
    }

    /// Show a different sheet (or, with `None`, the 3D model again).
    fn set_open_sheet(&mut self, id: Option<String>) {
        self.sheet_open = id;
    }

    /// The saved PMI view ids a placement may name, in block order.
    pub fn sheet_pmi_view_ids(&self) -> Vec<String> {
        self.pmi_state().views.iter().map(|view| view.id.clone()).collect()
    }

    /// The sheet object schemas, with the document's own PMI views as the
    /// `view` field's options.
    pub fn sheet_catalogue(&self) -> Value {
        crate::sheets::schema_catalogue(&self.sheet_pmi_view_ids())
    }

    /// The `__brepSheets` automation blob: the block, which sheet is open,
    /// which object's form is open, and the OPEN sheet's projection.
    ///
    /// Assembled rather than `json!`-built, because the projection is the big
    /// part and it is already serialised in the cache: everything this adds is
    /// O(sheets), never O(model). The registry publishes it every frame.
    pub fn sheet_state_json(&mut self) -> String {
        let state = self.sheet_state();
        let drawing = match self.sheet_open.clone() {
            Some(id) if self.sheet_project(&id) => self
                .sheet_cache
                .as_ref()
                .map(|cache| cache.drawing_json.as_str())
                .unwrap_or("null"),
            _ => "null",
        };
        let sheets = serde_json::to_string(&state.sheets).unwrap_or_else(|_| "[]".into());
        let open = serde_json::to_string(&self.sheet_open).unwrap_or_else(|_| "null".into());
        let object = serde_json::to_string(&self.sheet_open_object).unwrap_or_else(|_| "null".into());
        let selected = serde_json::to_string(&self.sheet_selected_object()).unwrap_or_else(|_| "null".into());
        format!(
            "{{\"sheets\":{sheets},\"idCounter\":{},\"openSheet\":{open},\"openObject\":{object},\"selectedObject\":{selected},\"drawing\":{drawing}}}",
            state.id_counter
        )
    }

    // --- the projection ------------------------------------------------------

    /// Project sheet `id` into the cache unless it is already there. `false`
    /// when the block carries no such sheet.
    ///
    /// The pair with [`Self::sheet_drawing_cached`] is what keeps the sheet
    /// viewport off a per-frame COPY of the drawing: the lookup needs `&mut`
    /// (it may project), the paint needs only `&`, and a caller that wants
    /// both at once used to satisfy the borrow by cloning the whole projection
    /// every frame.
    pub fn sheet_project(&mut self, id: &str) -> bool {
        let state = self.sheet_state();
        let Some(sheet) = state.find_sheet(id).cloned() else {
            return false;
        };
        // The cache key is the sheet, the run the scene came from, the
        // DOCUMENT's revision — every door that writes the saved document
        // bumps that counter, so no edit can miss it — which bodies are
        // SHOWN, and the title block's stamp. A camera orbit touches none of
        // them.
        let context = self.sheet_frame_context();
        self.request_sheet_topology(&sheet);
        self.pump_sheet_lines();
        let key = self.sheet_cache_key(id, &context);
        if self.sheet_cache.as_ref().is_none_or(|cache| cache.key != key || cache.reproject) {
            let pmi = self.pmi_state();
            // Every exact pass the cache does not hold goes to the RUNNER; the
            // placement draws its mesh approximation until the answer lands
            // and moves the key. An inline runner answers inside the call.
            let env = self.sheet_env();
            let mut complete = true;
            let (runner, work) = (&mut self.runner, &mut self.sheet_lines);
            let mut defer = |job: LinesJob| {
                let answer = work.defer(runner.as_mut(), job);
                complete &= !matches!(answer, Deferred::Pending);
                answer
            };
            let model = Model { scene: &self.scene, unposed: &self.pmi_explode_originals, env: &env };
            let mut drawing = project_sheet_with(
                &model,
                &sheet,
                &pmi.views,
                self.pmi_report.as_ref(),
                &context,
                &mut LinesMode::Defer(&mut defer),
            );
            self.draw_sheet_bom(&sheet, &mut drawing);
            let drawing_json = serde_json::to_string(&drawing).unwrap_or_else(|_| "null".into());
            // An answer that landed DURING the projection moved the key. The
            // drawing is stored under the key it ends on, so this frame's
            // lookup finds it and paints it. A complete drawing already holds
            // the answer; an incomplete one may have drawn that placement
            // before it landed, so the next call re-projects over it.
            let landed = self.sheet_cache_key(id, &context);
            let reproject = !complete && landed != key;
            let key = landed;
            self.sheet_cache = Some(SheetCache { key, drawing, drawing_json, complete, reproject });
        }
        true
    }

    /// The document's expression sheet, which a PMI dimension's tolerance may
    /// be written in — what a sheet dimension that inherits it reads.
    fn sheet_env(&self) -> brep_kernel::Env {
        brep_kernel::Env::build(&self.history.expressions(), &self.history.configurator()).unwrap_or_default()
    }

    /// Land every sheet pass the runner has answered (see [`SheetLinesWork`]).
    pub fn pump_sheet_lines(&mut self) {
        self.sheet_lines.pump(self.runner.as_mut());
    }

    /// Whether a sheet's exact pass is on the runner or waiting for it — the
    /// frame loop and the idle contract wait on it like a topology request.
    pub fn sheet_lines_pending(&self) -> bool {
        self.sheet_lines.pending()
    }

    /// Ask the runner for the topology of every solid `sheet` draws that the
    /// scene lacks (see [`Self::request_exact_solids`]). Until it lands the
    /// placement is drawn by the mesh approximation and says so; the arrival
    /// bumps the scene's exact revision, which is in the cache key.
    fn request_sheet_topology(&mut self, sheet: &crate::sheets::Sheet) {
        // The per-frame case: every kernel solid already held or asked for.
        let lacking = self.scene.solids().iter().any(|solid| {
            solid.source_handle != 0
                && !solid.is_sketch
                && !self.topology_asked.contains(&solid.source_handle)
                && self.scene.exact_solid(&solid.name).is_none()
        });
        if !lacking {
            self.pump_topology();
            return;
        }
        let pmi = self.pmi_state();
        let names = crate::sheets::project::sheet_solid_names(&self.scene, sheet, &pmi.views);
        self.request_exact_solids(names.iter().map(String::as_str));
    }

    /// What the title block prints beside the sheet's own fields: the document's
    /// name and today's date.
    fn sheet_frame_context(&self) -> FrameContext {
        FrameContext { document: self.document_name.clone(), date: today_iso() }
    }

    fn sheet_cache_key(
        &self,
        id: &str,
        context: &FrameContext,
    ) -> (String, u64, u64, u64, u64, u64, String) {
        (
            id.to_string(),
            self.applied_generation,
            self.history.revision(),
            self.scene.exact_revision(),
            // A placement draws the solids VISIBLE in the scene, and hiding one
            // is session state: it writes no history, runs nothing and lands no
            // exact solid, so without this term every term of the key stood
            // still while the drawing's content changed.
            self.scene.visibility_revision(),
            self.sheet_lines.revision,
            format!("{}\u{1f}{}", context.document, context.date),
        )
    }

    /// The CACHED projection of sheet `id`, without projecting: `None` unless
    /// [`Self::sheet_project`] put that sheet in the cache and nothing has
    /// invalidated it since (the whole key is checked, not just the id).
    pub fn sheet_drawing_cached(&self, id: &str) -> Option<&SheetDrawing> {
        let key = self.sheet_cache_key(id, &self.sheet_frame_context());
        self.sheet_cache
            .as_ref()
            .filter(|cache| cache.key == key)
            .map(|cache| &cache.drawing)
    }

    /// The projected sheet `id`, built on demand and cached until the model
    /// rebuilds or the block changes. `None` when the block carries no such
    /// sheet. The one-call door for an export; a PAINT uses the split pair
    /// above.
    pub fn sheet_drawing(&mut self, id: &str) -> Option<&SheetDrawing> {
        if !self.sheet_project(id) {
            return None;
        }
        self.sheet_cache.as_ref().map(|cache| &cache.drawing)
    }

    /// Sheet `id` as SVG — one of the sheet export's two output contracts.
    pub fn export_sheet_svg(&mut self, id: &str) -> Result<String, String> {
        self.plugin_export_ready()?;
        let id = self.resolve_sheet(id)?;
        let drawing = self
            .sheet_drawing_exact(&id)
            .ok_or_else(|| format!("no sheet '{id}'"))?;
        Ok(crate::sheets::svg::to_svg(drawing))
    }

    /// [`Self::sheet_drawing`] with every exact pass it lacks run HERE: a file
    /// is never written from a mesh approximation that only stood in while a
    /// pass was on the runner. The finished drawing replaces the cached one,
    /// so the viewport shows what was exported.
    pub fn sheet_drawing_exact(&mut self, id: &str) -> Option<&SheetDrawing> {
        if !self.sheet_project(id) {
            return None;
        }
        if self.sheet_cache.as_ref().is_some_and(|cache| !cache.complete) {
            let state = self.sheet_state();
            let sheet = state.find_sheet(id)?;
            let pmi = self.pmi_state();
            let env = self.sheet_env();
            let model = Model { scene: &self.scene, unposed: &self.pmi_explode_originals, env: &env };
            let mut drawing = project_sheet_with(
                &model,
                sheet,
                &pmi.views,
                self.pmi_report.as_ref(),
                &self.sheet_frame_context(),
                &mut LinesMode::Compute,
            );
            self.draw_sheet_bom(&sheet, &mut drawing);
            let drawing_json = serde_json::to_string(&drawing).unwrap_or_else(|_| "null".into());
            if let Some(cache) = self.sheet_cache.as_mut() {
                cache.drawing = drawing;
                cache.drawing_json = drawing_json;
                cache.complete = true;
                cache.reproject = false;
            }
        }
        self.sheet_cache.as_ref().map(|cache| &cache.drawing)
    }

    /// Sheet `id` as a one-page PDF titled with the sheet's name — the
    /// one-page case of [`Self::export_sheets_pdf`]. What the PDF cannot carry
    /// that the SVG does is written down in [`crate::sheets::pdf`].
    pub fn export_sheet_pdf(&mut self, id: &str) -> Result<Vec<u8>, String> {
        let id = self.resolve_sheet(id)?;
        let name = self.sheet_state().find_sheet(&id).map(|sheet| sheet.name.clone()).unwrap_or_default();
        self.export_sheets_pdf(&[id], &name)
    }

    /// EVERY sheet of the document as one PDF, in the block's sheet order —
    /// one page per sheet, each at its own paper size — titled with the
    /// document's name (the first sheet's when the document has none). This is
    /// what the export doors write; the SVG stays one file per sheet. The plain
    /// 2D drawing set: a placement's `threeD` is the 3D export's alone.
    pub fn export_document_pdf(&mut self) -> Result<Vec<u8>, String> {
        self.export_document_pdf_with(false)
    }

    /// The same drawing set with the live 3D model ([`crate::sheets::pdf3d`]):
    /// a 3D box over every placement marked `threeD`, and a last page given
    /// over to the model with a button per saved PMI view — Sheets + 3D (PDF).
    pub fn export_document_pdf_3d(&mut self) -> Result<Vec<u8>, String> {
        self.export_document_pdf_with(true)
    }

    fn export_document_pdf_with(&mut self, three_d: bool) -> Result<Vec<u8>, String> {
        let state = self.sheet_state();
        if state.sheets.is_empty() {
            return Err("the document has no sheets".into());
        }
        let ids: Vec<String> = state.sheets.iter().map(|sheet| sheet.id.clone()).collect();
        let title = if self.document_name.trim().is_empty() {
            state.sheets[0].name.clone()
        } else {
            self.document_name.clone()
        };
        self.export_sheets_pdf_with(&ids, &title, three_d)
    }

    /// Sheets `ids` as one PDF, one page each in the order given.
    ///
    /// The projection cache holds ONE sheet, the one on screen, and an export
    /// must not evict it: a sheet the cache already holds is read from it, and
    /// every other is projected for this export alone and dropped, so closing
    /// the export leaves the viewport exactly as warm as it was.
    pub fn export_sheets_pdf(&mut self, ids: &[String], title: &str) -> Result<Vec<u8>, String> {
        self.export_sheets_pdf_with(ids, title, false)
    }

    /// [`Self::export_sheets_pdf`], with the 3D model when `three_d`: over
    /// every placement marked `threeD`, and on a last page. Without it the
    /// file is [`crate::sheets::pdf::write_pdf`]'s, byte for byte, whatever
    /// the placements say.
    pub fn export_sheets_pdf_with(&mut self, ids: &[String], title: &str, three_d: bool) -> Result<Vec<u8>, String> {
        self.plugin_export_ready()?;
        let state = self.sheet_state();
        for id in ids {
            if let Some(sheet) = state.find_sheet(id) {
                self.request_sheet_topology(sheet);
            }
        }
        let context = self.sheet_frame_context();
        let pmi = self.pmi_state();
        let mut drawings: Vec<SheetDrawing> = Vec::with_capacity(ids.len());
        for id in ids {
            let sheet = state.find_sheet(id).ok_or_else(|| format!("no sheet '{id}'"))?;
            let complete = self.sheet_cache.as_ref().is_some_and(|cache| cache.complete);
            let drawing = match self.sheet_drawing_cached(id).filter(|_| complete) {
                Some(cached) => cached.clone(),
                None => {
                    let mut drawing = project_sheet_with(
                        &Model { scene: &self.scene, unposed: &self.pmi_explode_originals, env: &self.sheet_env() },
                        sheet, &pmi.views, self.pmi_report.as_ref(), &context, &mut LinesMode::Compute,
                    );
                    self.draw_sheet_bom(sheet, &mut drawing);
                    drawing
                },
            };
            drawings.push(drawing);
        }
        let pages: Vec<&SheetDrawing> = drawings.iter().collect();
        if !three_d {
            return Ok(crate::sheets::pdf::write_pdf(title, &pages));
        }
        let three_d = self.pdf3d_content(ids, &drawings, title)?;
        Ok(crate::sheets::pdf::write_pdf_3d(title, &pages, Some(&three_d)))
    }

    /// The 3D content of a sheet PDF: the model, a live box over each `threeD`
    /// placement that projected (one that did not — no camera, a perspective
    /// view — has no drawing to stand over and is left 2D), and the 3D page.
    fn pdf3d_content(
        &self,
        ids: &[String],
        drawings: &[SheetDrawing],
        title: &str,
    ) -> Result<crate::sheets::pdf3d::Pdf3d, String> {
        use crate::sheets::pdf3d::{self, Page3d, Placement3d};
        let state = self.sheet_state();
        let pmi = self.pmi_state();
        let bodies = self.pdf3d_bodies();
        let model = pdf3d::build(&bodies, &pmi, self.pmi_report.as_ref())?;
        // The live box stands over the drawing with a margin, centred on the
        // camera target's point so the model turns about the box's centre.
        const MARGIN_MM: f64 = 4.0;
        let mut placements = Vec::new();
        for (page, (id, drawing)) in ids.iter().zip(drawings).enumerate() {
            let Some(sheet) = state.find_sheet(id) else { continue };
            for placed in sheet.views.iter().filter(|placed| placed.three_d) {
                let Some(view) = drawing.views.iter().find(|view| view.id == placed.id) else { continue };
                if !view.error.is_empty() || model.view(&placed.view).is_none() {
                    continue;
                }
                let [x0, y0, x1, y1] = view.bounds;
                let p = placed.position;
                let half = [
                    ((p[0] - x0).max(x1 - p[0]) + MARGIN_MM).max(10.0),
                    ((p[1] - y0).max(y1 - p[1]) + MARGIN_MM).max(10.0),
                ];
                placements.push(Placement3d {
                    page,
                    id: placed.id.clone(),
                    view: placed.view.clone(),
                    centre_mm: p,
                    half_mm: half,
                    scale: placed.scale,
                });
            }
        }
        let page = Some({
            let (w, h) = state
                .find_sheet(ids.first().map(String::as_str).unwrap_or(""))
                .map(|sheet| sheet.millimetres())
                .unwrap_or((297.0, 210.0));
            let mut page = Page3d::layout(w.max(h), w.min(h), title, &model);
            if let Some((poster, scale)) = self.pdf3d_poster(&model, &page, &pmi) {
                page.poster = Some(poster);
                page.poster_scale = Some(scale);
            }
            page
        });
        Ok(pdf3d::Pdf3d { model, placements, page })
    }

    /// The 3D page's drawing of its default view: the view projected as a
    /// sheet placement at the centre of the 3D box, scaled to fill it — and
    /// the scale, which the box then opens at. A perspective view is drawn
    /// through the orthographic camera that frames its target alike (a sheet
    /// draws orthographic views only); `None` when the view does not project.
    fn pdf3d_poster(
        &self,
        model: &crate::sheets::pdf3d::Model3d,
        page: &crate::sheets::pdf3d::Page3d,
        pmi: &brep_kernel::PmiState,
    ) -> Option<(crate::sheets::project::ViewDrawing, f64)> {
        let default = model.views.get(page.default_view)?;
        let mut camera = default.camera.clone();
        if let brep_kernel::PmiProjection::Perspective { fov_y_deg } = camera.projection {
            let distance = {
                let d = [camera.target[0] - camera.eye[0], camera.target[1] - camera.eye[1], camera.target[2] - camera.eye[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            camera.projection = brep_kernel::PmiProjection::Orthographic {
                half_height: distance * (fov_y_deg.to_radians() * 0.5).tan(),
            };
        }
        let mut views = pmi.views.clone();
        match views.iter_mut().find(|view| view.id == default.id) {
            Some(view) => view.camera = Some(camera),
            None => views.push(brep_kernel::PmiView {
                id: default.id.clone(),
                name: default.name.clone(),
                camera: Some(camera),
                display: Default::default(),
                annotations: Vec::new(),
            }),
        }
        let centre = page.box_centre_mm();
        let half = page.box_half_mm();
        let project = |scale: f64| -> Option<crate::sheets::project::ViewDrawing> {
            let mut sheet: crate::sheets::Sheet = serde_json::from_value(serde_json::json!({
                "id": "MODEL3D",
                "size": crate::sheets::CUSTOM,
                "widthMm": page.width_mm,
                "heightMm": page.height_mm,
                "border": false,
                "titleBlock": false,
            }))
            .ok()?;
            sheet.views.push(PlacedView {
                id: "MODEL3D".into(),
                view: default.id.clone(),
                position: centre,
                scale,
                projection: ORTHOGRAPHIC.to_string(),
                flatten_text: true,
                three_d: false,
                section: None,
                detail: None,
            });
            let drawing = project_sheet_with(
                &Model { scene: &self.scene, unposed: &self.pmi_explode_originals, env: &self.sheet_env() },
                &sheet,
                &views,
                self.pmi_report.as_ref(),
                &self.sheet_frame_context(),
                &mut LinesMode::Compute,
            );
            drawing.views.into_iter().next().filter(|view| view.error.is_empty())
        };
        // Once at unit scale to measure, once at the scale that fills the box.
        let probe = project(1.0)?;
        let [x0, y0, x1, y1] = probe.bounds;
        let reach = [(centre[0] - x0).max(x1 - centre[0]), (centre[1] - y0).max(y1 - centre[1])];
        if !(reach[0] > 1e-9 && reach[1] > 1e-9) {
            return None;
        }
        let scale = (half[0] / reach[0]).min(half[1] / reach[1]) * 0.9;
        project(scale).map(|drawing| (drawing, scale))
    }

    /// The sheet an export writes: the named one, else the OPEN one, else the
    /// first in the block.
    pub fn resolve_sheet(&self, id: &str) -> Result<String, String> {
        let state = self.sheet_state();
        let id = id.trim();
        if !id.is_empty() {
            return state
                .find_sheet(id)
                .map(|sheet| sheet.id.clone())
                .ok_or_else(|| format!("no sheet '{id}'"));
        }
        if let Some(open) = self.sheet_open.as_ref().filter(|open| state.find_sheet(open).is_some()) {
            return Ok(open.clone());
        }
        state
            .sheets
            .first()
            .map(|sheet| sheet.id.clone())
            .ok_or_else(|| "the document has no sheets".to_string())
    }

    // --- block writes --------------------------------------------------------

    /// Write `state` as the document's block (checkpointed). An empty block is
    /// removed so a part that never had a sheet saves byte-identically.
    fn write_sheet_state(&mut self, state: SheetState, coalesce_key: Option<&str>) {
        let block = if state.is_empty() { None } else { serde_json::to_value(&state).ok() };
        self.history.set_sheets_block(block, coalesce_key);
        self.dirty = true;
    }

    // --- sheets ---------------------------------------------------------------

    /// Add a sheet (`name` or `Sheet N`, the default paper size) and open it.
    /// Returns the id.
    pub fn sheet_add(&mut self, name: Option<&str>) -> String {
        let mut state = self.sheet_state();
        let id = state.next_id("SHEET");
        let name = name
            .map(str::trim)
            .filter(|name| !name.is_empty())
            .map(String::from)
            .unwrap_or_else(|| format!("Sheet {}", state.sheets.len() + 1));
        let (width_mm, height_mm) = crate::sheets::paper_size(DEFAULT_SIZE).unwrap_or((420.0, 297.0));
        state.sheets.push(Sheet {
            id: id.clone(),
            name,
            size: DEFAULT_SIZE.to_string(),
            width_mm,
            height_mm,
            border: true,
            border_inset_mm: DEFAULT_BORDER_INSET_MM,
            title_block: true,
            notes: String::new(),
            views: Vec::new(),
            dimensions: Vec::new(),
            ordinates: Vec::new(),
            revisions: Vec::new(),
            bom_table: None,
        });
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(id.clone()));
        id
    }

    /// Insert a live BOM table on the open sheet and open its settings.
    pub fn sheet_insert_bom(&mut self) -> Result<(), String> {
        let id = self
            .sheet_open()
            .ok_or("open a drawing sheet first")?
            .to_owned();
        if self
            .sheet_state()
            .find_sheet(&id)
            .and_then(|s| s.bom_table.as_ref())
            .is_none()
        {
            self.sheet_update(
                &id,
                &serde_json::json!({ "bomTable": crate::sheets::BomTable::default() }).to_string(),
            )?;
        }
        self.sheet_set_object_open(Some(&id));
        Ok(())
    }

    /// Available built-in and custom attributes, in stable order.
    pub fn sheet_bom_columns(&self) -> Vec<String> {
        let mut columns = std::collections::BTreeSet::from([
            "partName".to_owned(),
            "occurrence.Quantity".to_owned(),
            "occurrence.Find_Number".to_owned(),
            "part.Part_Number".to_owned(),
            "part.Description".to_owned(),
            "part.Material".to_owned(),
        ]);
        for component in &self.assembly_components {
            for (scope, attrs) in [
                ("part", self.part_attributes(&component.part_name)),
                ("occurrence", self.occurrence_attributes(&component.id)),
            ] {
                if let Some(attrs) = attrs.as_object() {
                    columns.extend(attrs.keys().map(|key| format!("{scope}.{key}")));
                }
            }
        }
        columns.into_iter().collect()
    }

    fn draw_sheet_bom(&self, sheet: &Sheet, drawing: &mut SheetDrawing) {
        let Some(table) = &sheet.bom_table else {
            return;
        };
        let mut rows: std::collections::BTreeMap<Vec<String>, usize> =
            std::collections::BTreeMap::new();
        let occurrences = self.occurrence_attributes_all();
        let mut parts = std::collections::HashMap::new();
        for component in &self.assembly_components {
            let part = parts
                .entry(component.part_name.clone())
                .or_insert_with(|| self.part_attributes(&component.part_name));
            let occurrence = occurrences.get(&component.id).unwrap_or(&Value::Null);
            let cells: Vec<String> = table
                .columns
                .iter()
                .map(|column| {
                    if column == "partName" {
                        return component.part_name.clone();
                    }
                    if column == "quantity" || column == "occurrence.Quantity" {
                        return String::new();
                    }
                    let (scope, key) = column.split_once('.').unwrap_or(("part", column));
                    if column == "part.PMI" {
                        return self.part_pmi_count(&component.part_name).to_string();
                    }
                    let value = if scope == "occurrence" {
                        &occurrence[key]
                    } else {
                        &part[key]
                    };
                    match value {
                        Value::Null => String::new(),
                        Value::String(s) => s.clone(),
                        v => v.to_string(),
                    }
                })
                .collect();
            // Include the part identity even when its name column is hidden.
            let mut key = cells;
            key.push(component.part_name.clone());
            *rows.entry(key).or_default() += 1;
        }
        for wire in self.wire_bom_lines() {
            let mut cells: Vec<String> = table
                .columns
                .iter()
                .map(|column| match column.as_str() {
                    "partName" | "part.Part_Number" => wire.stock_part_number.clone(),
                    "occurrence.MF_QTY" => wire
                        .mf_qty
                        .map(|q| crate::formatting::compact_decimal(q, 3))
                        .unwrap_or_else(|| wire.state.as_str().to_owned()),
                    "occurrence.Reference_Designator" => wire.connection_id.clone(),
                    _ => String::new(),
                })
                .collect();
            cells.push(format!("wire:{}", wire.connection_id));
            rows.insert(cells, 1);
        }
        let frame = drawing
            .frame
            .get_or_insert_with(|| crate::sheets::frame::FrameDrawing {
                lines: Vec::new(),
                texts: Vec::new(),
                revision_table: None,
            });
        let [x, y] = table.position;
        let width = table.column_width_mm;
        let height = 7.;
        let count = rows.len() + 1;
        for row in 0..=count {
            frame.lines.push(vec![
                [x, y + row as f64 * height],
                [
                    x + table.columns.len() as f64 * width,
                    y + row as f64 * height,
                ],
            ]);
        }
        for col in 0..=table.columns.len() {
            frame.lines.push(vec![
                [x + col as f64 * width, y],
                [x + col as f64 * width, y + count as f64 * height],
            ]);
        }
        let headers: Vec<String> = table
            .columns
            .iter()
            .map(|c| match c.as_str() {
                "partName" => "Part name".into(),
                "quantity" => "Quantity".into(),
                _ => c
                    .split_once('.')
                    .map_or(c.as_str(), |(_, key)| key)
                    .replace('_', " "),
            })
            .collect();
        let mut all = vec![headers];
        for (mut cells, quantity) in rows {
            cells.pop();
            for (i, column) in table.columns.iter().enumerate() {
                if column == "quantity" || column == "occurrence.Quantity" {
                    cells[i] = quantity.to_string();
                }
            }
            all.push(cells);
        }
        for (row, cells) in all.iter().enumerate() {
            for (col, cell) in cells.iter().enumerate() {
                let available = ((width - 2.) / (2.5 * 0.65)).floor() as usize;
                let text = if cell.chars().count() > available {
                    format!(
                        "{}…",
                        cell.chars()
                            .take(available.saturating_sub(1))
                            .collect::<String>()
                    )
                } else {
                    cell.clone()
                };
                frame.texts.push(crate::sheets::project::SheetText::new(
                    text,
                    [
                        x + (col as f64 + 0.5) * width,
                        y + (row as f64 + 0.5) * height,
                    ],
                    [2.5, 0.],
                    [0., -2.5],
                ));
            }
        }
    }

    /// Apply an edited **Sheet** form.
    pub fn sheet_update(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        let params: Value = serde_json::from_str(params_json)
            .map_err(|error| format!("sheet params: {error}"))?;
        let mut state = self.sheet_state();
        let sheet = state.find_sheet_mut(id).ok_or_else(|| format!("no sheet '{id}'"))?;
        sheet.apply(&params)?;
        self.write_sheet_state(state, None);
        Ok(())
    }

    // --- the revision table ---------------------------------------------------

    /// Append a revision row to sheet `id` and return its index. Absent
    /// fields are seeded the way a drawing office starts a row: the letter
    /// after the last row's (`A` on an empty table, skipping `I`, `O` and `Q`
    /// as a view letter does), today's date, and no description.
    pub fn sheet_add_revision(
        &mut self,
        id: &str,
        rev: Option<&str>,
        date: Option<&str>,
        description: Option<&str>,
    ) -> Result<usize, String> {
        let mut state = self.sheet_state();
        let sheet = state.find_sheet_mut(id).ok_or_else(|| format!("no sheet '{id}'"))?;
        let next = || {
            let letters = crate::sheets::SectionCut::LETTERS;
            let last = sheet.revisions.last().and_then(|row| row.rev.trim().chars().next());
            match last.and_then(|c| letters.iter().position(|l| l.eq_ignore_ascii_case(&c))) {
                Some(at) => letters[(at + 1).min(letters.len() - 1)].to_string(),
                None if sheet.revisions.is_empty() => letters[0].to_string(),
                None => String::new(),
            }
        };
        let revision = crate::sheets::Revision {
            rev: rev.map(str::to_string).unwrap_or_else(next),
            date: date.map(str::to_string).unwrap_or_else(today_iso),
            description: description.unwrap_or_default().to_string(),
        };
        sheet.revisions.push(revision);
        let index = sheet.revisions.len() - 1;
        self.write_sheet_state(state, None);
        Ok(index)
    }

    /// Patch revision row `index` of sheet `id` (`rev`, `date`,
    /// `description`; absent keys are unchanged).
    pub fn sheet_update_revision(&mut self, id: &str, index: usize, params_json: &str) -> Result<(), String> {
        let params: Value = serde_json::from_str(params_json)
            .map_err(|error| format!("revision params: {error}"))?;
        let mut state = self.sheet_state();
        let sheet = state.find_sheet_mut(id).ok_or_else(|| format!("no sheet '{id}'"))?;
        let count = sheet.revisions.len();
        let row = sheet
            .revisions
            .get_mut(index)
            .ok_or_else(|| format!("sheet '{id}' has {count} revision row(s); there is no row {index}"))?;
        row.apply(&params)?;
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Remove revision row `index` of sheet `id`. The rows after it move up
    /// one and keep their order.
    pub fn sheet_remove_revision(&mut self, id: &str, index: usize) -> Result<(), String> {
        let mut state = self.sheet_state();
        let sheet = state.find_sheet_mut(id).ok_or_else(|| format!("no sheet '{id}'"))?;
        if index >= sheet.revisions.len() {
            return Err(format!(
                "sheet '{id}' has {} revision row(s); there is no row {index}",
                sheet.revisions.len()
            ));
        }
        sheet.revisions.remove(index);
        self.write_sheet_state(state, None);
        Ok(())
    }

    pub fn sheet_delete(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.sheet_state();
        let before = state.sheets.len();
        state.sheets.retain(|sheet| sheet.id != id);
        if state.sheets.len() == before {
            return Err(format!("no sheet '{id}'"));
        }
        if self.sheet_open.as_deref() == Some(id) {
            self.set_open_sheet(state.sheets.first().map(|sheet| sheet.id.clone()));
        }
        self.clear_open_object(&state);
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Show sheet `id` in the sheet viewport (`None` returns to the 3D view).
    pub fn sheet_set_open(&mut self, id: Option<&str>) -> Result<(), String> {
        match id {
            None => {
                self.set_open_sheet(None);
                self.dirty = true;
                Ok(())
            }
            Some(id) => {
                let state = self.sheet_state();
                if state.find_sheet(id).is_none() {
                    return Err(format!("no sheet '{id}'"));
                }
                self.set_open_sheet(Some(id.to_string()));
                self.dirty = true;
                Ok(())
            }
        }
    }

    /// Open a sheet or placed view's form in the Sheets pane (`None` closes).
    /// An open COUNTS ([`Self::open_sheet_object`]) — including a re-open of
    /// the object already open, which is a click on the same dimension on the
    /// paper while the Sheets pane sits behind another tab.
    pub fn sheet_set_object_open(&mut self, id: Option<&str>) {
        match id {
            Some(id) => self.open_sheet_object(id.to_string()),
            None => self.sheet_open_object = None,
        }
    }

    // --- placed views ----------------------------------------------------------

    /// Place PMI view `view_id` on sheet `sheet_id` (the open sheet when
    /// absent), centred on the paper unless `position` says otherwise, and
    /// open its form. Returns the placement id.
    pub fn sheet_place_view(
        &mut self,
        sheet_id: Option<&str>,
        view_id: &str,
        position: Option<[f64; 2]>,
        scale: Option<f64>,
    ) -> Result<String, String> {
        let pmi = self.pmi_state();
        let view = pmi
            .find_view(view_id)
            .ok_or_else(|| format!("no PMI view '{view_id}'"))?;
        let camera = view
            .camera
            .as_ref()
            .ok_or_else(|| format!("PMI view '{view_id}' has no captured camera — capture one first"))?;
        refuse_perspective(view_id, camera)?;
        let mut state = self.sheet_state();
        let sheet_id = match sheet_id.map(str::trim).filter(|id| !id.is_empty()) {
            Some(id) => id.to_string(),
            None => self.resolve_sheet("")?,
        };
        let id = state.next_id("SV");
        let sheet = state
            .find_sheet_mut(&sheet_id)
            .ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
        let (width_mm, height_mm) = sheet.millimetres();
        let scale = match scale {
            Some(scale) if scale.is_finite() && scale > 0.0 => scale,
            Some(scale) => return Err(format!("scale: {scale} is not a positive ratio")),
            None => DEFAULT_SCALE,
        };
        sheet.views.push(PlacedView {
            id: id.clone(),
            view: view_id.to_string(),
            position: position.unwrap_or([width_mm * 0.5, height_mm * 0.5]),
            scale,
            projection: ORTHOGRAPHIC.to_string(),
            flatten_text: true,
            three_d: false,
            section: None,
            detail: None,
        });
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(sheet_id));
        self.open_sheet_object(id.clone());
        Ok(id)
    }

    /// Apply an edited **Placed view** form.
    pub fn sheet_update_view(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        let mut params: Value = serde_json::from_str(params_json)
            .map_err(|error| format!("placed view params: {error}"))?;
        // A section's or a detail's PICKED references — the form's reference
        // rows — go through their own door, which checks them against the
        // sheet; and only when they changed, so an unrelated edit (the letter,
        // the scale) does not re-write the construction.
        if let Some(object) = params.as_object_mut() {
            let current = self.sheet_state().sheets.iter().find_map(|sheet| sheet.find_view(id).cloned());
            for field in ["cut", "centre", "rim"] {
                let Some(value) = object.remove(field) else { continue };
                // A single reference row hands back a string, a list row an array.
                let names: Vec<String> = match value.as_str() {
                    Some(one) => vec![one.to_string()],
                    None => crate::json_support::string_values(Some(&value)).map(str::to_string).collect(),
                }
                .into_iter()
                .filter(|name| !name.is_empty())
                .collect();
                let now = current.as_ref().map(|placed| placed.picked(field)).unwrap_or_default();
                if names != now {
                    self.sheet_set_placement_refs(id, field, &names)?;
                }
            }
        }
        // Re-pointing a placement at another saved view is a PLACEMENT of that
        // view, and refused on the same terms as the first one.
        if let Some(view_id) = params.get("view").and_then(Value::as_str).map(str::trim).filter(|id| !id.is_empty()) {
            if let Some(camera) = self.pmi_state().find_view(view_id).and_then(|view| view.camera.clone()) {
                refuse_perspective(view_id, &camera)?;
            }
        }
        let views = self.sheet_pmi_view_ids();
        let mut state = self.sheet_state();
        let view = state.find_view_mut(id).ok_or_else(|| format!("no placed view '{id}'"))?;
        view.apply(&params, &views)?;
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Move a placed view to `position` (paper millimetres). `coalesce` groups
    /// a drag into ONE undo step, the PMI label drag's rule.
    pub fn sheet_move_view(&mut self, id: &str, position: [f64; 2], coalesce: bool) -> Result<(), String> {
        if !position[0].is_finite() || !position[1].is_finite() {
            return Err("position: not a finite point".into());
        }
        let mut state = self.sheet_state();
        let view = state.find_view_mut(id).ok_or_else(|| format!("no placed view '{id}'"))?;
        view.position = position;
        let key = format!("sheet:view:{id}");
        self.write_sheet_state(state, coalesce.then_some(key.as_str()));
        Ok(())
    }

    /// End a placement drag: the next move starts a fresh undo entry.
    pub fn sheet_move_view_end(&mut self) {
        self.history.break_coalescing();
    }

    pub fn sheet_remove_view(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.sheet_state();
        let Some((sheet_id, index)) = state.locate_view(id) else {
            return Err(format!("no placed view '{id}'"));
        };
        if let Some(sheet) = state.find_sheet_mut(&sheet_id) {
            sheet.views.remove(index);
        }
        self.clear_open_object(&state);
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Close the open form when the object it showed is gone.
    fn clear_open_object(&mut self, state: &SheetState) {
        if let Some(open) = self.sheet_open_object.clone() {
            if state.find_sheet(&open).is_none()
                && state.locate_view(&open).is_none()
                && state.locate_dimension(&open).is_none()
                && state.locate_ordinate(&open).is_none()
            {
                self.sheet_open_object = None;
            }
        }
    }

    // --- sheet dimensions ------------------------------------------------------

    /// Add a dimension to sheet `sheet_id` (the open sheet when absent) and
    /// open its form. `anchors` are reference strings; an EMPTY list is
    /// allowed and gives an unresolved dimension whose form is where its
    /// anchors get picked.
    pub fn sheet_add_dimension(
        &mut self,
        sheet_id: Option<&str>,
        kind: &str,
        alignment: Option<&str>,
        anchors: Vec<String>,
    ) -> Result<String, String> {
        let kind = KINDS
            .iter()
            .find(|known| known.eq_ignore_ascii_case(kind.trim()))
            .ok_or_else(|| format!("no dimension kind '{kind}'"))?;
        let alignment = match alignment.map(str::trim).filter(|a| !a.is_empty()) {
            None => ALIGNED,
            Some(wanted) => ALIGNMENTS
                .iter()
                .copied()
                .find(|known| known.eq_ignore_ascii_case(wanted))
                .ok_or_else(|| format!("no alignment '{wanted}'"))?,
        };
        for anchor in &anchors {
            SheetAnchor::parse(anchor)?;
        }
        let sheet_id = match sheet_id.map(str::trim).filter(|id| !id.is_empty()) {
            Some(id) => id.to_string(),
            None => self.resolve_sheet("")?,
        };
        let mut state = self.sheet_state();
        let id = state.next_id("SD");
        let sheet = state
            .find_sheet_mut(&sheet_id)
            .ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
        sheet
            .dimensions
            .push(SheetDimension::new(id.clone(), kind, alignment, anchors));
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(sheet_id));
        self.open_sheet_object(id.clone());
        Ok(id)
    }

    /// Apply an edited **Sheet dimension** form.
    pub fn sheet_update_dimension(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        let params: Value = serde_json::from_str(params_json)
            .map_err(|error| format!("sheet dimension params: {error}"))?;
        let mut state = self.sheet_state();
        let dimension = state
            .find_dimension_mut(id)
            .ok_or_else(|| format!("no sheet dimension '{id}'"))?;
        dimension.apply(&params)?;
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Move a dimension's line: `offset_mm` from its anchors, signed. What
    /// dragging its value box in the sheet viewport does, so `coalesce` groups
    /// a drag into ONE undo step exactly as a placement drag does.
    pub fn sheet_move_dimension(
        &mut self,
        id: &str,
        offset_mm: f64,
        coalesce: bool,
    ) -> Result<(), String> {
        if !offset_mm.is_finite() {
            return Err(format!("offsetMm: {offset_mm} is not a length"));
        }
        let mut state = self.sheet_state();
        let dimension = state
            .find_dimension_mut(id)
            .ok_or_else(|| format!("no sheet dimension '{id}'"))?;
        dimension.offset_mm = offset_mm;
        let key = format!("sheet:dim:{id}");
        self.write_sheet_state(state, coalesce.then_some(key.as_str()));
        Ok(())
    }

    /// End an offset drag: the next move starts a fresh undo entry.
    pub fn sheet_move_dimension_end(&mut self) {
        self.history.break_coalescing();
    }

    pub fn sheet_remove_dimension(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.sheet_state();
        let Some((sheet_id, index)) = state.locate_dimension(id) else {
            return Err(format!("no sheet dimension '{id}'"));
        };
        if let Some(sheet) = state.find_sheet_mut(&sheet_id) {
            sheet.dimensions.remove(index);
        }
        self.clear_open_object(&state);
        self.write_sheet_state(state, None);
        Ok(())
    }

    // --- ordinate sets ------------------------------------------------------

    /// Add an ordinate set to sheet `sheet_id` (the open sheet when absent)
    /// and open its form. `datum` may be empty: the set is then unresolved and
    /// its form is where the datum gets picked.
    pub fn sheet_add_ordinate(
        &mut self,
        sheet_id: Option<&str>,
        axis: &str,
        datum: Option<&str>,
        members: Vec<String>,
    ) -> Result<String, String> {
        let axis = AXES
            .iter()
            .find(|known| known.eq_ignore_ascii_case(axis.trim()))
            .ok_or_else(|| format!("no ordinate axis '{axis}'"))?;
        let datum = datum.map(str::trim).filter(|d| !d.is_empty()).unwrap_or_default();
        if !datum.is_empty() {
            SheetAnchor::parse(datum)?;
        }
        for member in &members {
            SheetAnchor::parse(member)?;
        }
        let sheet_id = match sheet_id.map(str::trim).filter(|id| !id.is_empty()) {
            Some(id) => id.to_string(),
            None => self.resolve_sheet("")?,
        };
        let mut state = self.sheet_state();
        let id = state.next_id("OD");
        let sheet = state
            .find_sheet_mut(&sheet_id)
            .ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
        let mut set = SheetOrdinate::new(id.clone(), axis, datum.to_string());
        set.members = members;
        sheet.ordinates.push(set);
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(sheet_id));
        self.open_sheet_object(id.clone());
        Ok(id)
    }

    /// Apply an edited **Ordinate set** form.
    ///
    /// A set with no DATUM is kept, unresolved: it is what the Ordinate set
    /// button creates, and its form is where the datum gets picked. Every value
    /// it would draw is a distance from that datum, so until there is one the
    /// set draws its id and the reason instead.
    pub fn sheet_update_ordinate(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        let params: Value = serde_json::from_str(params_json)
            .map_err(|error| format!("ordinate set params: {error}"))?;
        let mut state = self.sheet_state();
        let set = state
            .find_ordinate_mut(id)
            .ok_or_else(|| format!("no ordinate set '{id}'"))?;
        set.apply(&params)?;
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Move a set's BASELINE: its signed distance from the datum in paper
    /// millimetres. One number for the whole set, so `coalesce` groups a drag
    /// into one undo step exactly as a dimension's offset drag does.
    pub fn sheet_move_ordinate(
        &mut self,
        id: &str,
        offset_mm: f64,
        coalesce: bool,
    ) -> Result<(), String> {
        if !offset_mm.is_finite() {
            return Err(format!("offsetMm: {offset_mm} is not a length"));
        }
        let mut state = self.sheet_state();
        let set = state
            .find_ordinate_mut(id)
            .ok_or_else(|| format!("no ordinate set '{id}'"))?;
        set.offset_mm = offset_mm;
        let key = format!("sheet:ord:{id}");
        self.write_sheet_state(state, coalesce.then_some(key.as_str()));
        Ok(())
    }

    /// End a baseline drag: the next move starts a fresh undo entry.
    pub fn sheet_move_ordinate_end(&mut self) {
        self.history.break_coalescing();
    }

    pub fn sheet_remove_ordinate(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.sheet_state();
        let Some((sheet_id, index)) = state.locate_ordinate(id) else {
            return Err(format!("no ordinate set '{id}'"));
        };
        if let Some(sheet) = state.find_sheet_mut(&sheet_id) {
            sheet.ordinates.remove(index);
        }
        self.clear_open_object(&state);
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// The baseline offset that would put set `id`'s baseline through the
    /// paper point `at`. `None` when the set has no datum to measure from.
    ///
    /// Unlike a dimension's, this needs no probe: a set's offset runs along
    /// ONE known paper axis — down the page for a horizontal set, across it
    /// for a vertical one — and the datum's own anchor point is the base.
    pub fn sheet_ordinate_offset_at(&mut self, id: &str, at: [f64; 2]) -> Option<f64> {
        let sheet_id = self.sheet_state().locate_ordinate(id).map(|(sheet, _)| sheet)?;
        let vertical = self
            .sheet_state()
            .find_ordinate(id)?
            .axis
            .eq_ignore_ascii_case(crate::sheets::dimension::VERTICAL);
        let drawing = self.sheet_drawing(&sheet_id)?;
        let drawn = drawing.ordinates.iter().find(|o| o.id == id)?;
        if !drawn.error.is_empty() {
            return None;
        }
        let datum = drawn.stations.iter().find(|station| station.datum)?;
        if !datum.error.is_empty() {
            return None;
        }
        Some(if vertical { at[0] - datum.at[0] } else { at[1] - datum.at[1] })
    }

    // --- section views ------------------------------------------------------

    /// Add a SECTION VIEW of `source`, cutting along the line through the two
    /// anchors, placed at `position` (beside the source when absent).
    ///
    /// The section places the SAME saved PMI view its source does — it needs
    /// that view's hidden-solid set and its text size — and overrides only the
    /// camera, which is derived from the cut rather than stored. Returns the
    /// placement id.
    pub fn sheet_add_section(
        &mut self,
        source: &str,
        from: &str,
        to: &str,
        position: Option<[f64; 2]>,
    ) -> Result<String, String> {
        SheetAnchor::parse(from)?;
        SheetAnchor::parse(to)?;
        if from == to {
            return Err("a cutting line needs two different anchors".into());
        }
        // Both ends belong to ONE placement: the line is drawn on that view
        // and it is that view's camera that turns a paper line into a plane.
        let (a, b) = (SheetAnchor::parse(from)?, SheetAnchor::parse(to)?);
        if a.view != b.view {
            return Err(format!(
                "'{from}' is on '{}' and '{to}' on '{}'; a cutting line is drawn on ONE view",
                a.view, b.view
            ));
        }
        if a.view != source {
            return Err(format!("the cutting line is on '{}', not on '{source}'", a.view));
        }
        let mut state = self.sheet_state();
        let Some((sheet_id, _)) = state.locate_view(source) else {
            return Err(format!("no placed view '{source}'"));
        };
        let (view, scale, at, label) = {
            let sheet = state
                .find_sheet(&sheet_id)
                .ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
            let label = sheet.next_view_letter();
            let parent = sheet
                .find_view(source)
                .ok_or_else(|| format!("no placed view '{source}'"))?;
            if parent.section.is_some() {
                return Err(format!(
                    "'{source}' is itself a section; a section of a section is not drawn"
                ));
            }
            if parent.detail.is_some() {
                return Err(format!("'{source}' is a detail; a section of a detail is not drawn"));
            }
            (parent.view.clone(), parent.scale, parent.position, label)
        };
        let id = state.next_id("SV");
        let sheet = state.find_sheet_mut(&sheet_id).expect("just found");
        sheet.views.push(PlacedView {
            id: id.clone(),
            view,
            // Beside the source by default, a paper width apart, because a
            // section drawn on top of the view it cuts is unreadable and the
            // pick that made it had nowhere to say where it goes.
            position: position.unwrap_or([at[0] + 120.0, at[1]]),
            scale,
            projection: ORTHOGRAPHIC.to_string(),
            flatten_text: true,
            three_d: false,
            section: Some(SectionCut {
                source: source.to_string(),
                from: from.to_string(),
                to: to.to_string(),
                label,
                flip: false,
            }),
            detail: None,
        });
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(sheet_id));
        self.open_sheet_object(id.clone());
        Ok(id)
    }

    /// Add a SECTION VIEW with no cutting line yet to sheet `sheet_id` (the
    /// open sheet when absent) and open its form — what the Drawing
    /// workbench's Section view button does. The placement is unresolved and
    /// says so on the paper; its form's **Cutting line** row is where the two
    /// anchors are picked ([`Self::sheet_set_placement_refs`]), and the first
    /// line picked decides which placement it is a section OF.
    pub fn sheet_new_section(&mut self, sheet_id: Option<&str>) -> Result<String, String> {
        self.sheet_new_derived(sheet_id, true)
    }

    /// The detail twin of [`Self::sheet_new_section`]: a DETAIL VIEW with no
    /// circle yet, whose form's **Centre** and **Point on the rim** rows are
    /// picked on a placed view.
    pub fn sheet_new_detail(&mut self, sheet_id: Option<&str>) -> Result<String, String> {
        self.sheet_new_derived(sheet_id, false)
    }

    fn sheet_new_derived(&mut self, sheet_id: Option<&str>, section: bool) -> Result<String, String> {
        let sheet_id = match sheet_id.map(str::trim).filter(|id| !id.is_empty()) {
            Some(id) => id.to_string(),
            None => self.resolve_sheet("")?,
        };
        let mut state = self.sheet_state();
        let id = state.next_id("SV");
        let sheet = state
            .find_sheet_mut(&sheet_id)
            .ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
        let label = sheet.next_view_letter();
        let (w, h) = sheet.millimetres();
        sheet.views.push(PlacedView {
            id: id.clone(),
            // No saved view until the first pick names a source: a section or
            // a detail draws its SOURCE's view.
            view: String::new(),
            position: [w * 0.5, h * 0.5],
            scale: DEFAULT_SCALE,
            projection: ORTHOGRAPHIC.to_string(),
            flatten_text: true,
            three_d: false,
            section: section.then(|| SectionCut {
                source: String::new(),
                from: String::new(),
                to: String::new(),
                label: label.clone(),
                flip: false,
            }),
            detail: (!section).then(|| DetailCircle {
                source: String::new(),
                centre: String::new(),
                rim: String::new(),
                label,
            }),
        });
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(sheet_id));
        self.open_sheet_object(id.clone());
        Ok(id)
    }

    /// Set a derived placement's PICKED references: `cut` (a section's two
    /// cutting-line anchors), `centre` or `rim` (a detail circle's). What a
    /// form's reference rows commit, the picker's Finish and a row's ✕ alike.
    ///
    /// The anchors must all lie on ONE plain placement of the same sheet — the
    /// line or circle is drawn on that view and read through its camera — and
    /// that placement becomes the source. A placement that has no saved view
    /// yet (made by [`Self::sheet_new_section`] / `_detail`) takes its
    /// source's view and scale — twice it, for a detail — and moves beside the
    /// source, where [`Self::sheet_add_section`] would have put it. Fewer
    /// anchors than the construction needs is allowed: the placement stays
    /// unresolved and says what is missing.
    pub fn sheet_set_placement_refs(&mut self, id: &str, field: &str, names: &[String]) -> Result<(), String> {
        for name in names {
            SheetAnchor::parse(name)?;
        }
        let mut state = self.sheet_state();
        let (sheet_id, _) = state.locate_view(id).ok_or_else(|| format!("no placed view '{id}'"))?;
        let sheet = state.find_sheet(&sheet_id).ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
        let placed = sheet.find_view(id).ok_or_else(|| format!("no placed view '{id}'"))?.clone();
        let single = |names: &[String]| -> Result<String, String> {
            match names {
                [] => Ok(String::new()),
                [one] => Ok(one.clone()),
                _ => Err(format!("'{field}' is one anchor")),
            }
        };
        // Every anchor of the construction after this edit, in pick order.
        let anchors: Vec<String> = match (field, &placed.section, &placed.detail) {
            ("cut", Some(_), _) => {
                if names.len() > 2 {
                    return Err("a cutting line is two anchors".into());
                }
                names.to_vec()
            }
            ("centre", _, Some(circle)) => vec![single(names)?, circle.rim.clone()],
            ("rim", _, Some(circle)) => vec![circle.centre.clone(), single(names)?],
            _ => return Err(format!("'{id}' has no {field} to pick")),
        };
        let picked: Vec<&String> = anchors.iter().filter(|a| !a.is_empty()).collect();
        if picked.len() == 2 && picked[0] == picked[1] {
            return Err(if placed.section.is_some() {
                "a cutting line needs two different anchors".into()
            } else {
                "a detail circle needs a radius: pick a rim point away from its centre".into()
            });
        }
        let on: Vec<String> = picked
            .iter()
            .map(|anchor| SheetAnchor::parse(anchor).map(|a| a.view))
            .collect::<Result<_, _>>()?;
        if on.windows(2).any(|pair| pair[0] != pair[1]) {
            return Err(format!(
                "'{}' is on '{}' and '{}' on '{}'; {} is drawn on ONE view",
                picked[0], on[0], picked[1], on[1],
                if placed.section.is_some() { "a cutting line" } else { "a detail circle" }
            ));
        }
        let source = on.first().cloned();
        let parent = match &source {
            Some(source) => {
                if source == id {
                    return Err(format!("'{id}' cannot be drawn on itself"));
                }
                let parent = sheet
                    .find_view(source)
                    .ok_or_else(|| format!("no placement '{source}' on this sheet"))?;
                let what = if placed.section.is_some() { "section" } else { "detail" };
                if parent.section.is_some() {
                    return Err(format!("'{source}' is a section; a {what} of a section is not drawn"));
                }
                if parent.detail.is_some() {
                    return Err(format!("'{source}' is a detail; a {what} of a detail is not drawn"));
                }
                Some((parent.view.clone(), parent.scale, parent.position))
            }
            None => None,
        };
        let sheet = state.find_sheet_mut(&sheet_id).expect("just found");
        let placed = sheet.views.iter_mut().find(|view| view.id == id).expect("just found");
        let source = source.unwrap_or_default();
        if let Some(cut) = placed.section.as_mut() {
            cut.from = anchors.first().cloned().unwrap_or_default();
            cut.to = anchors.get(1).cloned().unwrap_or_default();
            cut.source = source;
        } else if let Some(circle) = placed.detail.as_mut() {
            circle.centre = anchors[0].clone();
            circle.rim = anchors[1].clone();
            circle.source = source;
        }
        if let Some((view, scale, at)) = parent {
            if placed.view.is_empty() {
                placed.view = view;
                placed.scale = if placed.detail.is_some() { scale * DETAIL_SCALE_FACTOR } else { scale };
                placed.position = [at[0] + 120.0, at[1]];
            }
        }
        self.write_sheet_state(state, None);
        Ok(())
    }

    /// Enter the reference picker for sheet object `id`'s reference field
    /// `path` — a dimension's `anchors`, an ordinate set's `datum` or
    /// `members`, a section's `cut`, a detail's `centre` or `rim`. The same
    /// modal the feature, constraint and PMI dialogs use: the rest of the UI
    /// steps aside, picks land in the running list, **Finish** commits and
    /// **Cancel** discards. Here the picks are ANCHORS clicked on the paper, so
    /// the object's sheet is opened first, and nothing rolls.
    pub fn begin_ref_select_for_sheet(
        &mut self,
        id: &str,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed_names: Vec<String>,
    ) -> Result<(), String> {
        let state = self.sheet_state();
        let sheet = state
            .locate_view(id)
            .or_else(|| state.locate_dimension(id))
            .or_else(|| state.locate_ordinate(id))
            .map(|(sheet, _)| sheet)
            .ok_or_else(|| format!("no sheet object '{id}'"))?;
        self.set_open_sheet(Some(sheet));
        self.ref_select = Some(RefSelectState {
            feature_id: id.to_string(),
            path,
            label,
            filter,
            multiple,
            names: seed_names,
            restore_index: self.history.rollback(),
            target: RefSelectTarget::Sheet,
        });
        self.dirty = true;
        Ok(())
    }

    /// The most anchors the field being picked can hold, when the object
    /// says: a dimension's kind, a section's two ends. `None` = no limit.
    fn sheet_ref_capacity(&self, id: &str, field: &str) -> Option<usize> {
        let state = self.sheet_state();
        match field {
            "anchors" => state
                .find_dimension(id)
                .map(|dimension| crate::sheets::dimension::anchor_count(&dimension.kind)),
            "cut" => Some(2),
            _ => None,
        }
    }

    /// An anchor clicked on the paper while the sheet picker is up: added to
    /// the running list — replacing it in a single field, appended in a list
    /// (once), and in a list that is FULL for its object (a linear
    /// dimension's two, a cutting line's two) the oldest pick makes room, so a
    /// wrong pick is corrected by picking again.
    pub fn ref_select_pick_sheet_anchor(&mut self, anchor: &str) -> Result<(), String> {
        SheetAnchor::parse(anchor)?;
        let Some(state) = self.ref_select.as_ref().filter(|s| s.target == RefSelectTarget::Sheet) else {
            return Err("no sheet reference is being picked".into());
        };
        let capacity = state
            .path
            .last()
            .and_then(|field| self.sheet_ref_capacity(&state.feature_id, field));
        let state = self.ref_select.as_mut().expect("guarded above");
        if !state.multiple {
            state.names = vec![anchor.to_string()];
        } else if !state.names.iter().any(|name| name == anchor) {
            if capacity.is_some_and(|cap| cap > 0 && state.names.len() >= cap) {
                state.names.remove(0);
            }
            state.names.push(anchor.to_string());
        }
        self.dirty = true;
        Ok(())
    }

    /// Commit a finished sheet pick into its object — the one field it edits.
    pub(crate) fn sheet_commit_refs(&mut self, id: &str, path: &[String], names: &[String], multiple: bool) -> Result<(), String> {
        let field = path.last().map(String::as_str).unwrap_or_default();
        let value = if multiple {
            Value::Array(names.iter().cloned().map(Value::String).collect())
        } else {
            Value::String(names.first().cloned().unwrap_or_default())
        };
        let params = serde_json::json!({ field: value }).to_string();
        let state = self.sheet_state();
        if state.find_dimension(id).is_some() {
            self.sheet_update_dimension(id, &params)
        } else if state.find_ordinate(id).is_some() {
            self.sheet_update_ordinate(id, &params)
        } else if state.locate_view(id).is_some() {
            self.sheet_set_placement_refs(id, field, names)
        } else {
            Err(format!("no sheet object '{id}'"))
        }
    }

    // --- detail views ------------------------------------------------------

    /// Add a DETAIL VIEW of `source`: the region inside the circle centred on
    /// anchor `centre` and passing through anchor `rim`, redrawn at `scale`
    /// (twice the source's when absent) and placed at `position` (beside the
    /// source when absent).
    ///
    /// The detail places the SAME saved PMI view its source does and looks
    /// through the source's camera; its scale and its clip are its own.
    /// Returns the placement id.
    pub fn sheet_add_detail(
        &mut self,
        source: &str,
        centre: &str,
        rim: &str,
        position: Option<[f64; 2]>,
        scale: Option<f64>,
    ) -> Result<String, String> {
        let (a, b) = (SheetAnchor::parse(centre)?, SheetAnchor::parse(rim)?);
        if centre == rim {
            return Err("a detail circle needs two different anchors: its centre and a point on its rim".into());
        }
        // Both on ONE placement: the circle is drawn on that view, and it is
        // that view's camera the detail looks through.
        if a.view != b.view {
            return Err(format!(
                "'{centre}' is on '{}' and '{rim}' on '{}'; a detail circle is drawn on ONE view",
                a.view, b.view
            ));
        }
        if a.view != source {
            return Err(format!("the detail circle is on '{}', not on '{source}'", a.view));
        }
        if let Some(scale) = scale {
            if !scale.is_finite() || scale <= 0.0 {
                return Err(format!("scale: {scale} is not a positive ratio"));
            }
        }
        let mut state = self.sheet_state();
        let Some((sheet_id, _)) = state.locate_view(source) else {
            return Err(format!("no placed view '{source}'"));
        };
        let (view, parent_scale, at, label) = {
            let sheet = state
                .find_sheet(&sheet_id)
                .ok_or_else(|| format!("no sheet '{sheet_id}'"))?;
            let label = sheet.next_view_letter();
            let parent = sheet
                .find_view(source)
                .ok_or_else(|| format!("no placed view '{source}'"))?;
            // Refused BY NAME, both of them: the detail pass is one pass over
            // plain sources, so a chain is said out loud rather than drawn
            // half-resolved.
            if parent.detail.is_some() {
                return Err(format!("'{source}' is itself a detail; a detail of a detail is not drawn"));
            }
            if parent.section.is_some() {
                return Err(format!("'{source}' is a section; a detail of a section is not drawn"));
            }
            (parent.view.clone(), parent.scale, parent.position, label)
        };
        let id = state.next_id("SV");
        let sheet = state.find_sheet_mut(&sheet_id).expect("just found");
        sheet.views.push(PlacedView {
            id: id.clone(),
            view,
            // Beside the source by default, as a section is: a detail drawn on
            // top of the view it enlarges would hide the circle that names it.
            position: position.unwrap_or([at[0] + 120.0, at[1]]),
            scale: scale.unwrap_or(parent_scale * DETAIL_SCALE_FACTOR),
            projection: ORTHOGRAPHIC.to_string(),
            flatten_text: true,
            three_d: false,
            section: None,
            detail: Some(DetailCircle {
                source: source.to_string(),
                centre: centre.to_string(),
                rim: rim.to_string(),
                label,
            }),
        });
        self.write_sheet_state(state, None);
        self.set_open_sheet(Some(sheet_id));
        self.open_sheet_object(id.clone());
        Ok(id)
    }

    /// The offset that would put dimension `id`'s line through the paper point
    /// `at`: the component of `at - (the dimension's own base point)` along the
    /// direction the offset moves the line in. `None` when the dimension has
    /// not resolved — there is nothing to measure from.
    ///
    /// This is a PROBE of the current projection rather than a second layout:
    /// it re-reads the dimension's drawn label, which already sits at
    /// `base + offset`, so the answer is exact for any kind without this
    /// module knowing how any of them is constructed.
    pub fn sheet_dimension_offset_at(&mut self, id: &str, at: [f64; 2]) -> Option<f64> {
        let sheet_id = self.sheet_state().locate_dimension(id).map(|(sheet, _)| sheet)?;
        let drawing = self.sheet_drawing(&sheet_id)?;
        let drawn = drawing.dimensions.iter().find(|d| d.id == id)?;
        if !drawn.error.is_empty() {
            return None;
        }
        let (label, offset) = (drawn.label, drawn.offset_mm);
        // The label moves along ONE direction as the offset changes, so one
        // more sample of the same projection would be a second hidden-line
        // pass. Take the direction from the drawn geometry instead: the label
        // sits `offset` from the base along it.
        let direction = self.sheet_dimension_direction(&sheet_id, id)?;
        let base = [label[0] - direction[0] * offset, label[1] - direction[1] * offset];
        Some((at[0] - base[0]) * direction[0] + (at[1] - base[1]) * direction[1])
    }

    /// The unit paper direction dimension `id`'s offset moves its line along.
    fn sheet_dimension_direction(&mut self, sheet_id: &str, id: &str) -> Option<[f64; 2]> {
        // Re-lay the dimension out one millimetre further and read where the
        // label went: one projection is already cached, and the SECOND layout
        // is over the cached placements, not a fresh hidden-line pass.
        let state = self.sheet_state();
        let sheet = state.find_sheet(sheet_id)?;
        let mut probe = sheet.dimensions.iter().find(|d| d.id == id)?.clone();
        // Step AWAY from zero: a linear dimension measures its offset from
        // whichever anchor is farther from the side it is offset to, and that
        // choice turns over at zero — a probe that crossed it would read the
        // step between two different base points.
        let delta = if probe.offset_mm < 0.0 { -1.0 } else { 1.0 };
        probe.offset_mm += delta;
        let drawing = self.sheet_drawing_cached(sheet_id)?;
        let moved = crate::sheets::dimension::layout_dimensions(
            std::slice::from_ref(&probe),
            &drawing.views,
            [drawing.width_mm * 0.5, drawing.height_mm * 0.5],
        );
        let here = drawing.dimensions.iter().find(|d| d.id == id)?;
        let there = moved.first()?;
        if !there.error.is_empty() {
            return None;
        }
        let step = [
            (there.label[0] - here.label[0]) / delta,
            (there.label[1] - here.label[1]) / delta,
        ];
        let len = (step[0] * step[0] + step[1] * step[1]).sqrt();
        (len > 1e-9).then(|| [step[0] / len, step[1] / len])
    }

    /// Drop everything a sheet holds about the document being replaced.
    pub(crate) fn forget_sheets(&mut self) {
        self.sheet_open = None;
        self.sheet_open_object = None;
        self.sheet_selected_object = None;
        self.sheet_cache = None;
        self.sheet_lines.forget();
    }
}

/// Orthographic only, REFUSED at placement — by the placement door and by
/// the form that re-points a placement — rather than drawn flattened: a
/// perspective drawing is a different construction, not a worse one.
fn refuse_perspective(view_id: &str, camera: &brep_kernel::PmiCamera) -> Result<(), String> {
    if matches!(camera.projection, brep_kernel::PmiProjection::Orthographic { .. }) {
        Ok(())
    } else {
        Err(format!("PMI view '{view_id}' is a perspective camera; a sheet places {ORTHOGRAPHIC} views only"))
    }
}





