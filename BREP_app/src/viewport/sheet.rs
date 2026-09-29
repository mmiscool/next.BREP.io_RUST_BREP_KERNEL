//! The SHEET viewport mode — the viewport drawn as paper rather than as a 3D
//! scene.
//!
//! While the document has an OPEN sheet ([`EngineState::sheet_open`]) the
//! central tile draws that sheet's projection in two dimensions: the paper, its
//! border and title block, and each placed view's model lines and annotations,
//! all in sheet millimetres mapped through one pan/zoom transform. The 3D render is skipped
//! entirely — no offscreen pass, no blit — so a sheet costs a projection (cached
//! on the model's applied run and the block's revision) plus egui shapes.
//!
//! # There is no toolbar over the paper
//!
//! The sheet's TOOLS are workbench buttons, in the same toolbar row every other
//! mode's actions live in (`workbench/drawing.rs` declares them, conditional on a
//! sheet being open; `panels/toolbar.rs` draws the row). This tile draws paper
//! and drives the pointer — nothing else. In particular:
//!
//! * **Fit** is the MAIN toolbar's own Zoom-to-fit button, which frames the open
//!   sheet's paper instead of the 3D scene while a sheet is open —
//!   `BrepApp::zoom_to_fit` is that branch, and it reaches here through
//!   [`SheetViewport::request_fit`].
//! * **Zoom** is the wheel, about the cursor, off the SAME raw (unsmoothed)
//!   wheel delta the 3D viewport reads — one discrete step per notch, no
//!   ease-in ramp and no `+` / `−` buttons.
//! * **Back to 3D** is the `drawing.sheet_close` workbench button (and the Sheets
//!   pane's own **Close sheet** row action, which it has always been); leaving
//!   for a workbench without the Sheets pane closes an open sheet.
//! * The six **dimension constructions**, the two **ordinate sets**, the
//!   **section view** and the **detail view** are the `drawing.dim.*`,
//!   `drawing.ord.*`, `drawing.section` and `drawing.detail` workbench buttons.
//!   Each CREATES its object and opens its dialog; nothing is armed here.
//!
//! # Driving it
//!
//! Pan with a drag on empty paper (or the middle button anywhere), zoom with
//! the wheel about the cursor, and DRAG a placed view to move it on the paper —
//! one coalesced undo step per drag, the PMI label drag's rule. A press and
//! release without movement opens that placement's form in the Sheets pane.
//!
//! A RIGHT-click on a placement, a dimension's value box or an ordinate set's
//! datum box selects it and opens its MENU — the entries its Sheets tree row's
//! `⋯` offers, run through the same dispatch — and a right-click on bare paper
//! opens the sheet's own. The Delete key removes the selected object; that is
//! the app shell's (`BrepApp::delete_selected_annotation`), because the tree
//! selects the same objects.
//!
//! # Picking anchors on the paper
//!
//! What a sheet object measures or cuts is picked from its DIALOG's reference
//! rows, with the reference picker every dialog uses
//! ([`EngineState::begin_ref_select_for_sheet`]). While that picker is up the
//! shell draws this tile alone, every ANCHOR CANDIDATE of every placement is
//! marked on the paper and published as its own hit rect — a square on a
//! projected corner, a dot on a point of an edge or a circle's centre — the
//! anchors already picked drawn larger in the pick colour, and a click picks
//! the nearest, preferring a corner over a point on an edge the way the 3D
//! viewport prefers a vertex over an edge. A click there does nothing else: no
//! form opens and nothing drags; the paper still pans and zooms. The picker's
//! card, top right, lists the picks with Finish and Cancel; Escape cancels.
//!
//! A placed dimension's VALUE BOX is its handle: drag it to move the dimension
//! line (one coalesced undo step per drag, the placement drag's rule), click it
//! to SELECT the dimension and double-click it to open its form — the
//! annotation rule the PMI labels and every tree row follow; an ordinate set's
//! datum value box is the same handle for the set. A dimension whose anchor is
//! gone draws its id and the reason in the unresolved style instead of its
//! geometry.
//!
//! Every one of those is a published hit rect under the `sheet` panel — in
//! SCREEN points with no `panel:clip`, the way the `constraint` and `gizmo`
//! viewport widgets publish — so a script drives the sheet without a
//! coordinate: `sheet/paper`, one `sheet/view:<id>` per placement,
//! `sheet/dim:<dimension id>` per placed dimension, one
//! `sheet/menuitem:<object id>:<action id>` per entry of an open right-click
//! menu, and — only while the picker is up, because the list is as long as the
//! model's topology — one `sheet/anchor:<reference>` per candidate.

use super::*;
use brep_render::sheets::dimension::{DimensionDrawing, VERTEX};
use brep_render::sheets::ordinate::OrdinateDrawing;
use brep_render::sheets::project::{SheetDrawing, ViewDrawing};
use std::collections::HashMap;

/// What a frame of the sheet viewport asks the engine for, collected while the
/// PROJECTION is borrowed and applied once the borrow is gone.
///
/// The paint reads the cached projection by reference
/// ([`EngineState::sheet_drawing_cached`]) and every mutation on this tile —
/// a placement drag, a form open, an anchor pick — needs `&mut
/// EngineState`. Deferring them is what lets the paint borrow instead of
/// CLONING the whole drawing per frame, which is what this file used to do;
/// the visible behaviour is unchanged, because the paint already drew the
/// pre-mutation clone.
enum SheetIntent {
    /// Move a placement (coalesced: one undo step per drag).
    Move(String, [f64; 2]),
    /// The drag ended — the next move starts a fresh undo entry.
    MoveEnd,
    /// Open (or close, with `None`) a sheet object's form in the Sheets pane.
    OpenObject(Option<String>),
    /// Select a sheet annotation — a dimension or an ordinate set — without
    /// opening its form: a single click on its value box.
    SelectObject(String),
    /// Drag a dimension's value box: its new offset in paper millimetres,
    /// coalesced into one undo step per drag.
    MoveDimension(String, [f64; 2]),
    MoveDimensionEnd,
    /// Drag an ordinate set's baseline by its datum value box.
    MoveOrdinate(String, [f64; 2]),
    MoveOrdinateEnd,
    /// An anchor clicked while the sheet reference picker is up.
    RefPick(String),
}

/// The sheet reference picker, as this tile sees it: the anchors picked so
/// far (drawn in the pick colour) and the prompt written on the desk. Read
/// before the projection is borrowed, like everything else the paint needs.
#[derive(Debug, Clone, PartialEq)]
struct SheetPick {
    picked: Vec<String>,
    prompt: String,
}

/// What a press on the paper grabbed: a dimension's value box or an ordinate
/// set's datum box. Both are drawn OVER the placements and both are smaller
/// targets, so both are tested before a placement drag.
enum Grab {
    Dimension(String),
    Ordinate(String),
}

/// How close, in screen points, a click has to land to pick an anchor.
const ANCHOR_PICK_RADIUS: f32 = 9.0;

/// The marker a pickable anchor draws while the picker is up, screen points.
const ANCHOR_MARK: f32 = 2.2;

/// Millimetre margin left around the paper when the view is fitted.
const FIT_MARGIN: f32 = 1.06;
/// Wheel zoom per notch.
const ZOOM_STEP: f32 = 1.1;
/// Zoom limits, screen points per millimetre.
const MIN_ZOOM: f32 = 0.02;
const MAX_ZOOM: f32 = 200.0;
/// Model line width, points.
const EDGE_WIDTH: f32 = 1.1;
/// Annotation line width, points.
const ANNOTATION_WIDTH: f32 = 0.9;
/// Border and title-block line width, points.
const FRAME_WIDTH: f32 = 1.3;
/// A text run's cap height against its em — the ratio the SVG and PDF writers
/// convert with, so the screen and the files agree.
const CAP_PER_EM: f32 = 0.72;

/// The sheet viewport's transient state: which sheet the transform was fitted
/// for, the transform itself, and the live drag.
#[derive(Default)]
pub struct SheetViewport {
    /// The sheet the current pan/zoom belongs to; a different one refits.
    fitted: Option<String>,
    /// Screen points per sheet millimetre.
    zoom: f32,
    /// Screen point of the paper's top-left corner, relative to the tile.
    offset: egui::Vec2,
    /// A live pan drag.
    panning: bool,
    /// A live placement drag: the placement id and where inside it the
    /// pointer grabbed, in sheet millimetres.
    dragging: Option<(String, egui::Vec2)>,
    /// A live dimension OFFSET drag, by dimension id: the value box is the
    /// grab, and where the pointer goes is where the dimension line goes.
    dim_dragging: Option<String>,
    /// A live ordinate BASELINE drag, by set id: the datum's value box is the
    /// grab, and the whole run of values follows it.
    ord_dragging: Option<String>,
    /// The placement, dimension or ordinate set selected in the Sheets tree
    /// (`EngineState::sheet_selected_object`), read once a frame before the
    /// projection is borrowed; it is drawn in the accent a drag uses.
    selected: Option<String>,
    /// The open RIGHT-CLICK menu: the object it belongs to (a placement, a
    /// dimension, an ordinate set, or the sheet itself) and the screen point
    /// it opened at. Its entries are that object's Sheets tree row menu.
    menu: Option<(String, egui::Pos2)>,
    /// A right-click this frame asked for a menu — the object under the
    /// pointer, `None` for bare paper (the open sheet's own menu). Applied
    /// after the open menu has drawn, so the click that asked for a new menu
    /// is not also the click that closes it.
    menu_request: Option<(Option<String>, egui::Pos2)>,
    /// The rects published this frame (`sheet/…`, screen points).
    hits: HashMap<String, egui::Rect>,
}

impl SheetViewport {
    /// Refit the paper on the next paint — the toolbar's Zoom-to-fit while a
    /// sheet is open. Only the fitted MARKER is dropped, because the paint
    /// already refits whenever it does not name the open sheet; the live pan or
    /// drag and this frame's rects are untouched, which [`Self::forget`] would
    /// not leave alone.
    pub fn request_fit(&mut self) {
        self.fitted = None;
    }

    /// Forget the transform — a document switch, or leaving the sheet.
    pub fn forget(&mut self) {
        self.fitted = None;
        self.panning = false;
        self.dragging = None;
        self.dim_dragging = None;
        self.ord_dragging = None;
        self.menu = None;
        self.menu_request = None;
        self.hits.clear();
    }

    /// The `sheet/…` rects as `{key: [x, y, w, h]}` in screen points.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    fn to_screen(&self, rect: egui::Rect, mm: [f64; 2]) -> egui::Pos2 {
        rect.min + self.offset + egui::vec2(mm[0] as f32, mm[1] as f32) * self.zoom
    }

    fn to_sheet(&self, rect: egui::Rect, pos: egui::Pos2) -> egui::Vec2 {
        (pos - rect.min - self.offset) / self.zoom
    }

    fn fit(&mut self, rect: egui::Rect, width_mm: f64, height_mm: f64) {
        let (w, h) = (width_mm.max(1.0) as f32, height_mm.max(1.0) as f32);
        self.zoom = (rect.width() / (w * FIT_MARGIN))
            .min(rect.height() / (h * FIT_MARGIN))
            .clamp(MIN_ZOOM, MAX_ZOOM);
        self.offset = egui::vec2(
            (rect.width() - w * self.zoom) * 0.5,
            (rect.height() - h * self.zoom) * 0.5,
        );
    }

    /// Zoom about a screen point, so the paper under the cursor stays put.
    /// A placement's GRAB rect in screen points: its projected bounds, never
    /// thinner than a pointer can find — an empty or refused placement must
    /// still be draggable and clickable.
    fn grab_rect(&self, rect: egui::Rect, view: &ViewDrawing) -> egui::Rect {
        let bounds = egui::Rect::from_two_pos(
            self.to_screen(rect, [view.bounds[0], view.bounds[1]]),
            self.to_screen(rect, [view.bounds[2], view.bounds[3]]),
        );
        egui::Rect::from_center_size(
            bounds.center(),
            egui::vec2(bounds.width().max(12.0), bounds.height().max(12.0)),
        )
    }

    /// The topmost placement under `pos` — the LAST drawn wins, which is the
    /// one painted over the others.
    fn placement_at(
        &self,
        rect: egui::Rect,
        drawing: &brep_render::sheets::project::SheetDrawing,
        pos: egui::Pos2,
    ) -> Option<String> {
        drawing
            .views
            .iter()
            .rev()
            .find(|view| self.grab_rect(rect, view).contains(pos))
            .map(|view| view.id.clone())
    }

    /// The anchor candidate nearest `pos`, within [`ANCHOR_PICK_RADIUS`].
    ///
    /// A VERTEX wins over a circle centre and a circle centre over a point on
    /// an edge — the 3D viewport's own `VERTEX > EDGE` preference, read on the
    /// paper — because an edge's END candidate sits exactly on the vertex it
    /// runs to, and a corner is what a person clicking a corner means.
    fn anchor_at(
        &self,
        rect: egui::Rect,
        drawing: &SheetDrawing,
        pos: egui::Pos2,
    ) -> Option<String> {
        let mut best: Option<(u8, f32, &str)> = None;
        for view in &drawing.views {
            for candidate in &view.anchors {
                let at = self.to_screen(rect, candidate.at);
                let away = at.distance(pos);
                if away > ANCHOR_PICK_RADIUS {
                    continue;
                }
                let rank = match candidate.kind.as_str() {
                    VERTEX => 0u8,
                    brep_render::sheets::dimension::CIRCLE => 1,
                    _ => 2,
                };
                if best.is_none_or(|(r, d, _)| (rank, away) < (r, d)) {
                    best = Some((rank, away, candidate.anchor.as_str()));
                }
            }
        }
        best.map(|(_, _, anchor)| anchor.to_string())
    }

    /// A dimension's VALUE BOX in screen points — the grab that drags its
    /// offset, and the rect published as `sheet/dim:<id>`.
    fn value_rect(&self, rect: egui::Rect, dimension: &DimensionDrawing) -> egui::Rect {
        let centre = self.to_screen(rect, dimension.label);
        let (w, h) = match dimension.texts.first() {
            Some(run) => (
                run.text.chars().count() as f32 * 0.75 * run.height as f32 * self.zoom,
                run.height as f32 * self.zoom,
            ),
            None => (0.0, 0.0),
        };
        egui::Rect::from_center_size(centre, egui::vec2(w.max(14.0), h.max(12.0)))
    }

    /// The dimension whose value box is under `pos`.
    fn dimension_at(
        &self,
        rect: egui::Rect,
        drawing: &SheetDrawing,
        pos: egui::Pos2,
    ) -> Option<String> {
        drawing
            .dimensions
            .iter()
            .rev()
            .find(|dimension| self.value_rect(rect, dimension).contains(pos))
            .map(|dimension| dimension.id.clone())
    }

    /// An ordinate set's grab: the DATUM's value box, which is the one mark a
    /// whole run has in common — dragging it moves the shared baseline, and
    /// the published `sheet/ord:<id>` rect is that same box.
    fn ordinate_rect(&self, rect: egui::Rect, set: &OrdinateDrawing) -> egui::Rect {
        let centre = self.to_screen(rect, set.label);
        let datum = set.stations.iter().find(|station| station.datum);
        let run = datum.and_then(|station| {
            set.texts.iter().find(|text| text.text == station.text)
        });
        let (w, h) = match run {
            Some(run) => (
                run.text.chars().count() as f32 * 0.75 * run.height as f32 * self.zoom,
                run.height as f32 * self.zoom,
            ),
            None => (0.0, 0.0),
        };
        egui::Rect::from_center_size(centre, egui::vec2(w.max(14.0), h.max(12.0)))
    }

    /// The ordinate set whose datum value box is under `pos`.
    fn ordinate_at(
        &self,
        rect: egui::Rect,
        drawing: &SheetDrawing,
        pos: egui::Pos2,
    ) -> Option<String> {
        drawing
            .ordinates
            .iter()
            .rev()
            .find(|set| set.error.is_empty() && self.ordinate_rect(rect, set).contains(pos))
            .map(|set| set.id.clone())
    }

    fn zoom_about(&mut self, rect: egui::Rect, pivot: egui::Pos2, factor: f32) {
        let before = self.to_sheet(rect, pivot);
        self.zoom = (self.zoom * factor).clamp(MIN_ZOOM, MAX_ZOOM);
        let after = self.to_sheet(rect, pivot);
        self.offset += (after - before) * self.zoom;
    }
}

/// The sheet reference picker, when it is up — read BEFORE the projection is
/// borrowed. A 3D pick (a feature's or an annotation's reference) is not this
/// tile's, and never shows the anchors.
fn sheet_pick(state: &EngineState) -> Option<SheetPick> {
    state.ref_select_is_sheet().then(|| SheetPick {
        picked: state.ref_select_names(),
        prompt: format!("picking {}  \u{00b7}  click anchors, then Finish  \u{00b7}  Escape cancels", state.ref_select_prompt()),
    })
}

impl Viewport {
    /// Draw the OPEN sheet into `rect` and drive it. Returns `false` when the
    /// document has no open sheet, in which case the caller draws the 3D view.
    pub(super) fn show_sheet(
        &mut self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        response: &egui::Response,
        state: &mut EngineState,
    ) -> bool {
        let Some(open) = state.sheet_open().map(String::from) else {
            self.sheet.forget();
            return false;
        };
        // Project (cached) BEFORE anything borrows, so the paint below can read
        // the cache by reference and this tile costs nothing per frame beyond
        // its own painting.
        if !state.sheet_project(&open) {
            self.sheet.forget();
            return false;
        }
        let mut intents: Vec<SheetIntent> = Vec::new();
        // The picker is read BEFORE the projection is borrowed: the paint
        // needs to know whether to offer anchors, and the borrow below is what
        // keeps the drawing off a per-frame copy.
        let pick = sheet_pick(state);
        self.sheet.selected = state.sheet_selected_object().map(String::from);
        let Some(drawing) = state.sheet_drawing_cached(&open) else {
            self.sheet.forget();
            return false;
        };
        self.paint_and_drive_sheet(ui, rect, response, &open, drawing, pick.as_ref(), &mut intents);
        self.apply_sheet_intents(state, intents);
        self.show_sheet_menu(ui, state, &open);
        true
    }

    /// The RIGHT-CLICK menu, drawn once the projection is no longer borrowed:
    /// the object's Sheets tree row menu — the same entries its `⋯` offers,
    /// run through the same dispatch (`panels::sheets::run_row_action`), so
    /// the paper and the tree cannot offer different things. Its entries are
    /// published as `sheet/menuitem:<object id>:<action id>`.
    fn show_sheet_menu(&mut self, ui: &egui::Ui, state: &mut EngineState, open: &str) {
        let mut chosen: Option<(String, String)> = None;
        if let Some((id, pos)) = self.sheet.menu.clone() {
            let actions = crate::panels::sheets::object_actions(&state.sheet_state(), Some(open), &id);
            if actions.is_empty() {
                // The object went (deleted from the tree, undone) and took
                // its menu with it.
                self.sheet.menu = None;
            } else {
                let hits = &mut self.sheet.hits;
                let (action, still_open) = crate::column_tree::action_menu(
                    ui.ctx(),
                    ui.layer_id(),
                    egui::Id::new("brep-sheet-menu"),
                    pos,
                    &actions,
                    |entry, rect| {
                        hits.insert(format!("menuitem:{id}:{}", entry.id), rect);
                    },
                );
                if !still_open {
                    self.sheet.menu = None;
                }
                chosen = action.map(|action| (id, action));
            }
        }
        if let Some((id, action)) = chosen {
            if let Err(error) = crate::panels::sheets::run_row_action(state, &id, &action) {
                state.push_notice(format!("Sheets: {error}"));
            }
        }
        if let Some((object, pos)) = self.sheet.menu_request.take() {
            self.sheet.menu = Some((object.unwrap_or_else(|| open.to_string()), pos));
        }
    }

    /// One frame of the open sheet: fit, input, paint, toolbar — all against a
    /// BORROWED projection, with every engine mutation collected into
    /// `intents`.
    #[allow(clippy::too_many_arguments)]
    fn paint_and_drive_sheet(
        &mut self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        response: &egui::Response,
        open: &str,
        drawing: &SheetDrawing,
        pick: Option<&SheetPick>,
        intents: &mut Vec<SheetIntent>,
    ) {
        self.sheet.hits.clear();

        // --- the transform ------------------------------------------------------
        if self.sheet.fitted.as_deref() != Some(open) || self.sheet.zoom <= 0.0 {
            self.sheet.fit(rect, drawing.width_mm, drawing.height_mm);
            self.sheet.fitted = Some(open.to_string());
        }

        // --- input ----------------------------------------------------------------
        self.handle_sheet_input(ui, rect, response, drawing, pick.is_some(), intents);

        // --- paint ----------------------------------------------------------------
        let painter = ui.painter_at(rect);
        let visuals = ui.visuals();
        let desk = if visuals.dark_mode {
            egui::Color32::from_rgb(0x1b, 0x1f, 0x24)
        } else {
            egui::Color32::from_rgb(0x8a, 0x8f, 0x96)
        };
        painter.rect_filled(rect, 0.0, desk);
        let paper = egui::Rect::from_min_max(
            self.sheet.to_screen(rect, [0.0, 0.0]),
            self.sheet.to_screen(rect, [drawing.width_mm, drawing.height_mm]),
        );
        painter.rect_filled(paper, 0.0, egui::Color32::from_rgb(0xfa, 0xfa, 0xf7));
        painter.rect_stroke(
            paper,
            0.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(0x33, 0x33, 0x33)),
            egui::StrokeKind::Inside,
        );
        self.sheet.hits.insert("paper".into(), paper);

        let ink = egui::Color32::from_rgb(0x11, 0x11, 0x11);
        // The paper's own furniture first, under the drawing.
        if let Some(frame) = &drawing.frame {
            for line in &frame.lines {
                let points: Vec<egui::Pos2> =
                    line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
                painter.add(egui::Shape::line(points, egui::Stroke::new(FRAME_WIDTH, ink)));
            }
            for text in &frame.texts {
                self.paint_sheet_text(&painter, rect, text, ink);
            }
            if let Some(table) = &frame.revision_table {
                for line in &table.lines {
                    let points: Vec<egui::Pos2> =
                        line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
                    painter.add(egui::Shape::line(points, egui::Stroke::new(FRAME_WIDTH, ink)));
                }
                for text in &table.texts {
                    self.paint_sheet_text(&painter, rect, text, ink);
                }
            }
        }
        for view in &drawing.views {
            self.paint_sheet_view(ui, &painter, rect, view, ink);
        }
        // The sheet's OWN dimensions, over the placements — the writers' own
        // layering.
        for dimension in &drawing.dimensions {
            self.paint_sheet_dimension(&painter, rect, dimension, ink);
        }
        for set in &drawing.ordinates {
            self.paint_sheet_ordinate(&painter, rect, set, ink);
        }
        // While the picker is up, every anchor candidate of every placement is
        // offered: a marker to see it by and a keyed rect to click it with.
        // They are published only while it IS up, because the candidate list
        // is O(the model's topology) and this blob goes out every frame.
        if let Some(pick) = pick {
            self.paint_anchor_candidates(&painter, rect, drawing, &pick.picked);
        }
        self.paint_sheet_caption(&painter, rect, drawing, pick);
        let _ = ui;
    }

    /// What this tile is, written on the desk above the paper: the sheet's name
    /// and paper size, and — while the reference picker is up — what is being
    /// picked.
    ///
    /// This is PAINT, not a widget: no `Area`, no frame, nothing to click.
    fn paint_sheet_caption(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        drawing: &SheetDrawing,
        pick: Option<&SheetPick>,
    ) {
        let at = rect.min + egui::vec2(10.0, 8.0);
        painter.text(
            at,
            egui::Align2::LEFT_TOP,
            format!(
                "{}  \u{00b7}  {:.0} \u{00d7} {:.0} mm",
                drawing.name, drawing.width_mm, drawing.height_mm
            ),
            egui::FontId::proportional(12.0),
            egui::Color32::from_rgb(0xd0, 0xd4, 0xd8),
        );
        if let Some(pick) = pick {
            painter.text(
                at + egui::vec2(0.0, 16.0),
                egui::Align2::LEFT_TOP,
                &pick.prompt,
                egui::FontId::proportional(12.0),
                egui::Color32::from_rgb(0x58, 0xa6, 0xff),
            );
        }
    }

    /// Apply what the frame asked for, once the projection is no longer
    /// borrowed. A refusal reaches the user as the engine's own notice.
    fn apply_sheet_intents(&mut self, state: &mut EngineState, intents: Vec<SheetIntent>) {
        for intent in intents {
            match intent {
                SheetIntent::Move(id, position) => {
                    let _ = state.sheet_move_view(&id, position, true);
                }
                SheetIntent::MoveEnd => state.sheet_move_view_end(),
                SheetIntent::OpenObject(id) => {
                    // Empty paper closes the form AND drops the selection: the
                    // click picked nothing. (An object's open selects it.)
                    if id.is_none() {
                        state.sheet_select_object(None);
                    }
                    state.sheet_set_object_open(id.as_deref());
                }
                SheetIntent::SelectObject(id) => {
                    state.sheet_select_object(Some(&id));
                }
                SheetIntent::MoveDimension(id, at) => {
                    if let Some(offset) = state.sheet_dimension_offset_at(&id, at) {
                        let _ = state.sheet_move_dimension(&id, offset, true);
                    }
                }
                SheetIntent::MoveDimensionEnd => state.sheet_move_dimension_end(),
                SheetIntent::MoveOrdinate(id, at) => {
                    if let Some(offset) = state.sheet_ordinate_offset_at(&id, at) {
                        let _ = state.sheet_move_ordinate(&id, offset, true);
                    }
                }
                SheetIntent::MoveOrdinateEnd => state.sheet_move_ordinate_end(),
                SheetIntent::RefPick(anchor) => {
                    if let Err(error) = state.ref_select_pick_sheet_anchor(&anchor) {
                        state.push_notice(format!("Sheets: {error}"));
                    }
                }
            }
        }
    }

    fn paint_sheet_view(
        &mut self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        rect: egui::Rect,
        view: &ViewDrawing,
        ink: egui::Color32,
    ) {
        let grab = self.sheet.grab_rect(rect, view);
        self.sheet.hits.insert(format!("view:{}", view.id), grab);

        let dragged = self.sheet.dragging.as_ref().is_some_and(|(id, _)| *id == view.id)
            || self.sheet.selected.as_deref() == Some(view.id.as_str());
        if dragged || !view.error.is_empty() {
            let color = if view.error.is_empty() {
                egui::Color32::from_rgb(0x58, 0xa6, 0xff)
            } else {
                egui::Color32::from_rgb(0xf8, 0x51, 0x49)
            };
            painter.rect_stroke(
                grab.expand(2.0),
                2.0,
                egui::Stroke::new(1.0, color),
                egui::StrokeKind::Outside,
            );
        }
        if !view.error.is_empty() {
            painter.text(
                grab.center(),
                egui::Align2::CENTER_CENTER,
                &view.error,
                egui::FontId::proportional(11.0),
                egui::Color32::from_rgb(0xf8, 0x51, 0x49),
            );
            return;
        }
        for run in &view.edges {
            let points: Vec<egui::Pos2> = run.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
            painter.add(egui::Shape::line(points, egui::Stroke::new(EDGE_WIDTH, ink)));
        }
        // A placement drawn by the MESH APPROXIMATION says so on screen, as its
        // SVG and PDF do: its exact pass is on the runner (or its B-rep has not
        // reached this side), and the lines are the display's chords until it
        // lands. Paint, not a widget — there is nothing to click.
        if view.hidden_line == "mesh" {
            painter.text(
                grab.left_top() + egui::vec2(0.0, -3.0),
                egui::Align2::LEFT_BOTTOM,
                "approximate hidden lines (mesh)",
                egui::FontId::proportional(11.0),
                egui::Color32::from_rgb(0xb0, 0x70, 0x10),
            );
        }
        // A SECTION's cut face: the hatch, then the caption. The cut OUTLINE
        // is already above, in `edges` — the clip's new boundary goes through
        // the same hidden-line pass every other model line does.
        if let Some(section) = &view.section {
            for line in &section.hatch {
                let points: Vec<egui::Pos2> =
                    line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
                painter.add(egui::Shape::line(points, egui::Stroke::new(ANNOTATION_WIDTH, ink)));
            }
            for text in &section.texts {
                self.paint_sheet_text(painter, rect, text, ink);
            }
        }
        // …and the section LINES this placement carries for the sections cut
        // through it.
        for mark in &view.section_marks {
            for line in &mark.lines {
                let points: Vec<egui::Pos2> =
                    line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
                painter.add(egui::Shape::line(points, egui::Stroke::new(FRAME_WIDTH, ink)));
            }
            for text in &mark.texts {
                self.paint_sheet_text(painter, rect, text, ink);
            }
        }
        // A DETAIL's boundary circle and caption, and the detail CIRCLES this
        // placement carries for the details drawn from it — the writers' own
        // layering.
        if let Some(detail) = &view.detail {
            let points: Vec<egui::Pos2> =
                detail.circle.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
            painter.add(egui::Shape::line(points, egui::Stroke::new(ANNOTATION_WIDTH, ink)));
            for text in &detail.texts {
                self.paint_sheet_text(painter, rect, text, ink);
            }
        }
        for mark in &view.detail_marks {
            for line in &mark.lines {
                let points: Vec<egui::Pos2> =
                    line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
                painter.add(egui::Shape::line(points, egui::Stroke::new(FRAME_WIDTH, ink)));
            }
            for text in &mark.texts {
                self.paint_sheet_text(painter, rect, text, ink);
            }
        }
        for annotation in &view.annotations {
            for line in &annotation.lines {
                let points: Vec<egui::Pos2> = line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
                painter.add(egui::Shape::line(points, egui::Stroke::new(ANNOTATION_WIDTH, ink)));
            }
            for text in &annotation.texts {
                self.paint_sheet_text(painter, rect, text, ink);
            }
        }
        let _ = ui;
    }

    /// One SHEET dimension: its lines and its value, or — when its anchor is
    /// gone — its id and the reason, in the UNRESOLVED style the 3D overlay
    /// uses for an annotation that will not resolve. Never dropped: a drawing
    /// that silently loses a dimension is worse than one that says so.
    fn paint_sheet_dimension(
        &mut self,
        painter: &egui::Painter,
        rect: egui::Rect,
        dimension: &DimensionDrawing,
        ink: egui::Color32,
    ) {
        let box_rect = self.sheet.value_rect(rect, dimension);
        self.sheet.hits.insert(format!("dim:{}", dimension.id), box_rect);
        if !dimension.error.is_empty() {
            let red = egui::Color32::from_rgb(0xf8, 0x51, 0x49);
            painter.text(
                self.sheet.to_screen(rect, dimension.label),
                egui::Align2::CENTER_CENTER,
                format!("{}  \u{2014}  {}", dimension.id, dimension.error),
                egui::FontId::proportional(11.0),
                red,
            );
            return;
        }
        let dragged = self.sheet.dim_dragging.as_deref() == Some(dimension.id.as_str())
            || self.sheet.selected.as_deref() == Some(dimension.id.as_str());
        let colour = if dragged { egui::Color32::from_rgb(0x58, 0xa6, 0xff) } else { ink };
        for line in &dimension.lines {
            let points: Vec<egui::Pos2> = line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
            painter.add(egui::Shape::line(points, egui::Stroke::new(ANNOTATION_WIDTH, colour)));
        }
        for text in &dimension.texts {
            self.paint_sheet_text(painter, rect, text, colour);
        }
    }

    /// One ORDINATE SET: its leaders, its datum's origin circle and its run of
    /// values — or, when it has no datum left, its id and the reason in the
    /// unresolved style, exactly as a lost dimension reads.
    ///
    /// A set whose DATUM resolved but which lost a MEMBER still draws: the
    /// stations that resolved are inked and a red count under the datum's
    /// value says how many went, because a set that vanished when one of five
    /// points moved would be worse than one that says which point went.
    fn paint_sheet_ordinate(
        &mut self,
        painter: &egui::Painter,
        rect: egui::Rect,
        set: &OrdinateDrawing,
        ink: egui::Color32,
    ) {
        let red = egui::Color32::from_rgb(0xf8, 0x51, 0x49);
        if !set.error.is_empty() {
            painter.text(
                self.sheet.to_screen(rect, set.label),
                egui::Align2::CENTER_CENTER,
                format!("{}  \u{2014}  {}", set.id, set.error),
                egui::FontId::proportional(11.0),
                red,
            );
            return;
        }
        let box_rect = self.sheet.ordinate_rect(rect, set);
        self.sheet.hits.insert(format!("ord:{}", set.id), box_rect);
        let dragged = self.sheet.ord_dragging.as_deref() == Some(set.id.as_str())
            || self.sheet.selected.as_deref() == Some(set.id.as_str());
        let colour = if dragged { egui::Color32::from_rgb(0x58, 0xa6, 0xff) } else { ink };
        for line in &set.lines {
            let points: Vec<egui::Pos2> = line.iter().map(|p| self.sheet.to_screen(rect, *p)).collect();
            painter.add(egui::Shape::line(points, egui::Stroke::new(ANNOTATION_WIDTH, colour)));
        }
        for text in &set.texts {
            self.paint_sheet_text(painter, rect, text, colour);
        }
        let lost = set.lost();
        if lost > 0 {
            painter.text(
                box_rect.center_bottom() + egui::vec2(0.0, 2.0),
                egui::Align2::CENTER_TOP,
                format!("{lost} member{} lost", if lost == 1 { "" } else { "s" }),
                egui::FontId::proportional(10.0),
                red,
            );
        }
    }

    /// Every anchor a sheet object can be picked on, while the picker is up: a
    /// small mark at each and one keyed rect per candidate
    /// (`sheet/anchor:<reference>`), so a script picks an anchor by NAME —
    /// the way the 3D viewport's `click_entity` reaches a scene entity. The
    /// anchors already PICKED draw larger, in the pick colour, so the running
    /// list in the picker's card can be read off the paper too.
    fn paint_anchor_candidates(
        &mut self,
        painter: &egui::Painter,
        rect: egui::Rect,
        drawing: &SheetDrawing,
        picked: &[String],
    ) {
        let candidate_mark = egui::Color32::from_rgb(0x58, 0xa6, 0xff);
        let picked_mark = egui::Color32::from_rgb(0xff, 0x9f, 0x0a);
        for view in &drawing.views {
            for candidate in &view.anchors {
                let at = self.sheet.to_screen(rect, candidate.at);
                if !rect.contains(at) {
                    // A candidate scrolled off the tile publishes no rect: a
                    // rect egui will not accept a click on is a lie.
                    continue;
                }
                // A corner draws as a square and a point on an edge as a
                // dot, so the two ranks the picker prefers between are told
                // apart before the click rather than after it.
                let is_picked = picked.iter().any(|name| *name == candidate.anchor);
                let (mark, size) = if is_picked {
                    (picked_mark, ANCHOR_MARK * 2.0)
                } else {
                    (candidate_mark, ANCHOR_MARK)
                };
                if candidate.kind == VERTEX {
                    painter.rect_filled(
                        egui::Rect::from_center_size(at, egui::Vec2::splat(size * 2.0)),
                        0.0,
                        mark,
                    );
                } else {
                    painter.circle_filled(at, size, mark);
                }
                self.sheet.hits.insert(
                    format!("anchor:{}", candidate.anchor),
                    egui::Rect::from_center_size(at, egui::Vec2::splat(ANCHOR_PICK_RADIUS)),
                );
            }
        }
    }

    /// One placed text run — a dimension's value, a title block cell — centred
    /// on its anchor and rotated about it. The layout sizes a CAP height; an
    /// egui font size is the em.
    ///
    /// This is the run's `angle` and `height` — its basis SUMMARISED — and not
    /// the basis itself, because egui cannot shear a `TextShape`. Every run
    /// laid flat on the paper (the whole title block, and every annotation of
    /// a placement with **Flatten text** on, which is the default) is drawn
    /// exactly; a run projected out of an annotation plane oblique to the
    /// view previews as its rotation at its foreshortened height, while the
    /// exported SVG and PDF carry the full projected basis as a text matrix.
    ///
    /// A `TextShape`'s `pos` is the galley's TOP-LEFT, and
    /// `with_angle_and_anchor` only compensates the ROTATION about that anchor —
    /// it does not move the galley onto it. Centring is therefore this
    /// subtraction, without which every run on the paper drew half a text box
    /// down and to the right of where the drawing puts it.
    fn paint_sheet_text(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        text: &brep_render::sheets::project::SheetText,
        ink: egui::Color32,
    ) {
        let size = (text.height as f32 * self.sheet.zoom / CAP_PER_EM).clamp(1.0, 200.0);
        let galley =
            painter.layout_no_wrap(text.text.clone(), egui::FontId::proportional(size), ink);
        let centre = galley.rect.center().to_vec2();
        let mut shape = egui::epaint::TextShape::new(egui::Pos2::ZERO, galley, ink)
            .with_angle_and_anchor((text.angle as f32).to_radians(), egui::Align2::CENTER_CENTER);
        shape.pos += self.sheet.to_screen(rect, text.anchor).to_vec2() - centre;
        painter.add(shape);
    }

    #[allow(clippy::too_many_arguments)]
    fn handle_sheet_input(
        &mut self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        response: &egui::Response,
        drawing: &SheetDrawing,
        picking: bool,
        intents: &mut Vec<SheetIntent>,
    ) {
        let pointer = ui.ctx().pointer_interact_pos();
        // Whether a right-click menu was up coming into this frame: a click on
        // the paper then only DISMISSES it (the popup closes itself on any
        // click), rather than also opening a form or dropping the selection.
        let menu_up = self.sheet.menu.is_some();
        // Wheel: zoom about the cursor, off the RAW (unsmoothed) delta the 3D
        // viewport reads — one discrete step per notch, no ease-in/ease-out
        // ramp. The paper and the scene answer the wheel the same way, which is
        // the whole reason the paper needs no zoom buttons.
        let wheel = super::interaction::raw_wheel_delta_y(ui.ctx());
        if response.hovered() && wheel.abs() > 0.01 {
            let pivot = pointer.unwrap_or(rect.center());
            self.sheet.zoom_about(rect, pivot, ZOOM_STEP.powf(wheel / 50.0));
        }

        if response.drag_started() {
            // Where the press LANDED, not where the pointer is now: egui does
            // not report a drag until it has passed its threshold, and by then
            // the pointer has left a target as small as a dimension's value
            // box — which is how a value-box drag used to become a placement
            // drag.
            let start = ui
                .ctx()
                .input(|i| i.pointer.press_origin())
                .or(pointer)
                .unwrap_or(rect.center());
            let middle = ui.ctx().input(|i| i.pointer.middle_down());
            // A dimension's VALUE BOX is grabbed before a placement is: it is
            // drawn over the placements and it is the smaller target.
            // Nothing is grabbed while the picker is up: the paper is being
            // READ for anchors then, and a press on it pans.
            let grab = (!middle && !picking)
                .then(|| {
                    self.sheet
                        .dimension_at(rect, drawing, start)
                        .map(Grab::Dimension)
                        .or_else(|| self.sheet.ordinate_at(rect, drawing, start).map(Grab::Ordinate))
                })
                .flatten();
            // Falling THROUGH to the `dragged()` block below rather than
            // returning is what makes a short drag land: a press and its first
            // move arrive in the same frame, so a frame that only armed the
            // grab would throw that move away.
            if let Some(grab) = grab {
                match grab {
                    Grab::Dimension(id) => self.sheet.dim_dragging = Some(id),
                    Grab::Ordinate(id) => self.sheet.ord_dragging = Some(id),
                }
            } else {
                let hit = (!middle && !picking)
                    .then(|| self.sheet.placement_at(rect, drawing, start))
                    .flatten();
                match hit {
                    Some(id) => {
                        let position = drawing
                            .views
                            .iter()
                            .find(|view| view.id == id)
                            .map(|view| view.position)
                            .unwrap_or([0.0, 0.0]);
                        let grab = self.sheet.to_sheet(rect, start)
                            - egui::vec2(position[0] as f32, position[1] as f32);
                        self.sheet.dragging = Some((id, grab));
                    }
                    None => self.sheet.panning = true,
                }
            }
        }
        if response.dragged() {
            let delta = response.drag_delta();
            if let Some(id) = self.sheet.dim_dragging.clone() {
                if let Some(pos) = pointer {
                    let at = self.sheet.to_sheet(rect, pos);
                    intents.push(SheetIntent::MoveDimension(id, [at.x as f64, at.y as f64]));
                }
            } else if let Some(id) = self.sheet.ord_dragging.clone() {
                if let Some(pos) = pointer {
                    let at = self.sheet.to_sheet(rect, pos);
                    intents.push(SheetIntent::MoveOrdinate(id, [at.x as f64, at.y as f64]));
                }
            } else if self.sheet.panning {
                self.sheet.offset += delta;
            } else if let Some((id, grab)) = self.sheet.dragging.clone() {
                if let Some(pos) = pointer {
                    let target = self.sheet.to_sheet(rect, pos) - grab;
                    intents.push(SheetIntent::Move(id, [target.x as f64, target.y as f64]));
                }
            }
        }
        if response.drag_stopped() {
            self.sheet.panning = false;
            if self.sheet.dragging.take().is_some() {
                intents.push(SheetIntent::MoveEnd);
            }
            if self.sheet.dim_dragging.take().is_some() {
                intents.push(SheetIntent::MoveDimensionEnd);
            }
            if self.sheet.ord_dragging.take().is_some() {
                intents.push(SheetIntent::MoveOrdinateEnd);
            }
        }
        // A RIGHT-click opens the menu of what is under the pointer — the menu
        // its Sheets tree row's `⋯` opens — and selects it, so the object the
        // menu acts on carries the accent. The hit order is the left click's:
        // a value box over a placement. On bare paper it is the SHEET's own
        // menu. Nothing while the picker is up: the paper is being read for
        // anchors then.
        if response.secondary_clicked() && !picking {
            let at = pointer.unwrap_or(rect.center());
            let object = self
                .sheet
                .dimension_at(rect, drawing, at)
                .or_else(|| self.sheet.ordinate_at(rect, drawing, at))
                .or_else(|| self.sheet.placement_at(rect, drawing, at));
            if let Some(id) = &object {
                intents.push(SheetIntent::SelectObject(id.clone()));
            }
            self.sheet.menu_request = Some((object, at));
        }
        // A bare CLICK (no drag at all). While the picker is up it owns every
        // click on the paper: an anchor under the pointer is PICKED, and a
        // click anywhere else does nothing — it never opens a form behind the
        // picker's back.
        if response.clicked() && !menu_up {
            let start = pointer.unwrap_or(rect.center());
            if picking {
                if let Some(anchor) = self.sheet.anchor_at(rect, drawing, start) {
                    intents.push(SheetIntent::RefPick(anchor));
                }
                return;
            }
            // A dimension's or a set's value box is an ANNOTATION: a click
            // selects it and a double click opens its form. The double click is
            // read first, because egui reports its second press as a click too.
            let annotation = self
                .sheet
                .dimension_at(rect, drawing, start)
                .or_else(|| self.sheet.ordinate_at(rect, drawing, start));
            if let Some(id) = annotation {
                intents.push(if crate::panels::tree::is_double_click(&response) {
                    SheetIntent::OpenObject(Some(id))
                } else {
                    SheetIntent::SelectObject(id)
                });
                return;
            }
            // Then a placement, whose click opens its form; then empty paper,
            // which closes whatever was open.
            intents.push(SheetIntent::OpenObject(self.sheet.placement_at(rect, drawing, start)));
        }
    }
}
