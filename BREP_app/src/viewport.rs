//! The 3D viewport — everything that draws + drives the central `brep-render`
//! scene, split out of the thin app shell.
//!
//! [`Viewport`] owns the GPU seam between the engine and egui:
//!
//! * The engine's [`RenderCore`] renders the 3D into an app-owned OFFSCREEN
//!   texture (its own MSAA + full-target clear), on eframe's SHARED wgpu
//!   device/queue.
//! * That offscreen texture is composited into egui's own frame via an
//!   `egui_wgpu` paint callback ([`ViewportCallback`]/[`ViewportPaint`]) — a
//!   fullscreen-triangle blit into egui's render pass, at the viewport rect egui
//!   scissors for us — so the 3D and the egui UI share one wgpu frame.
//! * Pointer / wheel / ViewCube over the viewport route into `EngineState`
//!   (mirrors `desktop.rs`).
//!
//! [`Viewport::show`] is the single clean entry the shell calls to draw +
//! interact with the central viewport; [`EngineState`] stays the brain and is
//! borrowed in, never owned here.

use brep_render::controls::{BUTTON_LEFT, BUTTON_MIDDLE, BUTTON_RIGHT};
use brep_render::engine_state::EngineState;
use brep_render::pick::PickCandidate;
use brep_render::style::MultiSelectMode;
use brep_render::render::{FrameParams, GpuScene, RenderCore, COLOR_FORMAT};
use eframe::egui;
use eframe::egui_wgpu;
use std::sync::Arc;

/// GPU-side resources the callback needs, stored in egui's `callback_resources`
/// type-map. Refreshed whenever the offscreen texture is (re)created.
struct ViewportPaint {
    pipeline: wgpu::RenderPipeline,
    bind_group: wgpu::BindGroup,
}

/// The paint callback: a zero-sized, `Send + Sync` marker. All the heavy GPU
/// resources live in `callback_resources`; the actual 3D render already happened
/// (into the offscreen texture) during `App::ui`, so `paint` only blits.
struct ViewportCallback;

/// The app-owned offscreen color target the engine resolves into.
struct Offscreen {
    view: wgpu::TextureView,
    w: u32,
    h: u32,
}

/// The open PICK LIST — the "candidates under the cursor" popup that opens on a
/// plain click with MULTIPLE filter-admitted items under the pointer (and on
/// Alt+click explicitly): the ranked, filter-respecting candidates captured when
/// it opened + the screen anchor (cursor position) to draw the list at.
struct CandidatePopup {
    anchor: egui::Pos2,
    candidates: Vec<PickCandidate>,
}

/// The central 3D viewport: the engine's render core + the app-owned offscreen
/// texture + the egui blit that composites it into egui's frame, plus the
/// pointer/wheel/ViewCube routing. Borrows `&mut EngineState` to draw + drive;
/// never owns the engine brain.
pub struct Viewport {
    /// The engine's wgpu render core, built from eframe's device/queue/format.
    core: RenderCore,
    gpu_scene: GpuScene,
    /// egui's renderer, so we can push `ViewportPaint` into its callback map.
    egui_renderer: Arc<egui::mutex::RwLock<egui_wgpu::Renderer>>,
    blit_layout: wgpu::BindGroupLayout,
    blit_sampler: wgpu::Sampler,
    blit_pipeline: wgpu::RenderPipeline,
    offscreen: Option<Offscreen>,
    /// True while a camera drag (orbit/pan) started over the viewport is live.
    dragging: bool,
    touch: touch::TouchNavigation,
    touch_suppressed: bool,
    /// True while a transform-gizmo HANDLE drag (armed via the in-viewport center
    /// sphere toggle) is live — the pointer press landed on a handle, so the drag
    /// drives the gizmo (edits the feature's transform) instead of orbiting the camera.
    gizmo_dragging: bool,
    /// True while a COMPONENT Move gizmo handle drag is live (assemblies §8.5):
    /// the press landed on the armed component gizmo, so the drag free-moves the
    /// GIZMO ([`EngineState::component_drag_to`]) and the release COMMITS the
    /// composed pose + re-solves ([`EngineState::component_release`]) instead of
    /// orbiting the camera.
    component_gizmo_dragging: bool,
    /// `Some(field_key)` while a DIMENSION-ARROW drag is live (Fix 4) — the press
    /// landed on an arrowhead handle in ◎ dimension mode, so the drag edits that
    /// param via [`EngineState::feature_dimension_drag`] instead of orbiting the
    /// camera. `None` when no arrow drag is in flight.
    dim_dragging: Option<String>,
    /// True while a sketch POINT drag started over the viewport is live (S2) — the
    /// press grabbed a movable sketch point, so the drag moves it instead of
    /// orbiting the camera.
    sketch_dragging: bool,
    /// True while a freehand handdraw STROKE started over the viewport is live
    /// (S6b-3) — the press began a stroke, so the drag captures it as a polyline
    /// instead of orbiting the camera.
    sketch_handdrawing: bool,
    /// The viewport rect (egui points) from the last `show`, so the shell can
    /// publish the viewport origin for the headed verifier (which needs to turn
    /// the engine's viewport-local pick coords into page pixels).
    last_rect: Option<egui::Rect>,
    /// The open pick-list popup (a plain multi-candidate click / Alt+click), or
    /// None. Holds the ranked candidates snapshot + the cursor anchor.
    candidate_popup: Option<CandidatePopup>,
    /// True on the frame the popup opens, so the OPENING Alt+click isn't misread
    /// as a click-outside that would close it immediately.
    candidate_popup_fresh: bool,
    /// The HOVER DWELL clock: `ctx.input(|i| i.time)` at the moment the hover
    /// highlight the pointer is now showing FIRST appeared, or `None` while no
    /// highlight is lit. It is what tells the two plain-click behaviours apart —
    /// a click on a highlight that has been standing for the dwell opens the pick
    /// list, a quicker one selects the highlight outright — so the rule is a
    /// property of the thing the user can SEE, not of a hidden timer: the
    /// highlight's own age. Restarted whenever the highlight CHANGES
    /// (`EngineState::hover_at` reports it, which also covers the camera or the
    /// geometry moving under a still pointer) AND whenever the POINTER itself
    /// moves — so travelling across one big face, where the highlight never
    /// changes, accumulates no dwell. See
    /// `interaction::Viewport::hover_dwell_armed`.
    hover_lit_since: Option<f64>,
    /// Where the pointer was on the frame before, so the clock above can be
    /// restarted on real MOVEMENT. Deliberately not egui's own
    /// `time_since_last_movement`: that restamps on ANY `PointerMoved` event,
    /// including a redundant one carrying the position the pointer already had,
    /// which some input paths (and the automation queue, which re-sends a
    /// position before a click) emit — and a dwell a stationary pointer can lose
    /// to a no-op event is a dwell the user cannot reach on purpose.
    hover_last_pos: Option<egui::Pos2>,
    /// The OPEN popup's bounding rect (egui points) from the last draw, so the
    /// viewport click handler can tell a click ON the popup from one BEHIND it
    /// (an entry click belongs to the popup, not a scene pick).
    candidate_popup_rect: Option<egui::Rect>,
    /// Per-entry screen rects of the OPEN popup (index-aligned to its
    /// candidates), published for the headed verifier to click a specific entry.
    candidate_hits: Vec<egui::Rect>,
    /// The dimension label currently being edited (S5): its constraint id + the live
    /// text buffer, or `None` when no field is open. Clicking a label opens it,
    /// Enter applies, Esc cancels.
    editing_dim: Option<(serde_json::Value, String)>,
    /// True on the frame a dim edit field opens, so its seeded text gets focus
    /// before the click-outside logic can close it.
    dim_edit_fresh: bool,
    /// The FEATURE-dimension label currently being edited (FD-1): `(feature_id,
    /// field_key, live text buffer)`, or `None`. Mirrors `editing_dim` for the ◎
    /// dimension-gizmo mode. Clicking a dim label opens it, Enter applies via
    /// [`EngineState::feature_dimension_set_value`], Esc cancels; dragging drives
    /// [`EngineState::feature_dimension_drag`].
    editing_feature_dim: Option<(String, String, String)>,
    /// True on the frame a feature-dim edit field opens (focus seeding).
    feature_dim_edit_fresh: bool,
    /// True while an ASSEMBLY-CONSTRAINT handle drag is live — the press
    /// landed on a distance leader / angle-arc handle, so the drag previews
    /// that constraint's value via [`EngineState::constraint_drag_to`]
    /// (release commits + auto-solves) instead of orbiting the camera.
    constraint_dragging: bool,
    /// The constraint id whose LABEL the pointer hovered LAST frame (the
    /// element-highlight lane) — cleared through
    /// [`EngineState::constraint_hover_end`] when the pointer leaves the labels.
    constraint_label_hovered: Option<String>,
    /// The PMI label chip under the pointer (its referenced geometry is
    /// highlighted; ended exactly once when the pointer leaves).
    pmi_label_hovered: Option<String>,
    /// The PMI label chip being dragged (its world anchor follows the pointer
    /// on the label's depth plane; release ends the coalesced undo step).
    pmi_label_dragging: Option<String>,
    /// The PMI chips drawn LAST frame, `(annotation id, screen rect)` — the
    /// `__brepPmiLabelHit` verifier global (drag / click a chip by rect).
    pmi_label_hits: Vec<(String, egui::Rect)>,
    /// The open RIGHT-CLICK menu of a PMI label chip: the annotation it belongs
    /// to and the screen point it opened at. Its entries are the annotation's
    /// PMI tree row menu.
    pmi_label_menu: Option<(String, egui::Pos2)>,
    /// That menu's entries this frame, `menuitem:<annotation id>:<action id>`
    /// — the `__brepPmiMenuHit` verifier global.
    pmi_menu_hits: std::collections::HashMap<String, egui::Rect>,
    /// The SHEET mode's pan/zoom and live drag. While the document has an open
    /// sheet the tile draws paper instead of the 3D scene (`sheet.rs`).
    sheet: sheet::SheetViewport,
    /// The eCAD host's rects and card state while an eCAD workbench draws its
    /// editor here (`ecad.rs`). The editors themselves, and their views, live
    /// on the document.
    ecad: ecad::EcadViewport,
}

impl Viewport {
    /// Drop everything this viewport is holding ABOUT ONE DOCUMENT — called when
    /// the shell switches the active document (see `crate::document`).
    ///
    /// The GPU cache is the load-bearing half: [`GpuScene`] keys its retained
    /// buffers by SOLID NAME and reuses one while the solid's `revision` is
    /// unchanged, but revisions are per-`EngineState` and every document counts
    /// from the same place — so document B's "Box" rev 1 would be served
    /// document A's uploaded mesh. Only the active document renders, so a full
    /// re-upload on the switch is the right price. The transients after it
    /// (pick popup, open dimension editors, in-flight drags) all name entities
    /// of the document that is going away.
    pub fn forget_document(&mut self) {
        self.gpu_scene = GpuScene::default();
        self.close_candidate_popup();
        self.candidate_hits.clear();
        self.editing_dim = None;
        self.editing_feature_dim = None;
        self.constraint_label_hovered = None;
        self.pmi_label_hovered = None;
        self.pmi_label_dragging = None;
        self.pmi_label_hits.clear();
        self.pmi_label_menu = None;
        self.pmi_menu_hits.clear();
        self.sheet.forget();
        self.ecad = ecad::EcadViewport::default();
        self.dragging = false;
        self.gizmo_dragging = false;
        self.component_gizmo_dragging = false;
        self.dim_dragging = None;
        self.sketch_dragging = false;
        self.sketch_handdrawing = false;
        self.constraint_dragging = false;
    }

    /// Refit the OPEN SHEET's paper on this frame's paint — what the toolbar's
    /// Zoom-to-fit does while a sheet is open (see `BrepApp::zoom_to_fit`).
    /// Clears ONLY the fitted marker, so a live pan or drag and the rects
    /// published this frame all survive.
    pub fn request_sheet_fit(&mut self) {
        self.sheet.request_fit();
    }

    /// Drop the sheet viewport's transform and transients — the caller has just
    /// closed the sheet, so the next frame draws the 3D scene.
    pub fn forget_sheet(&mut self) {
        self.sheet.forget();
    }

    /// Close the pick-list popup if open; returns whether one WAS open. The app
    /// shell's global Escape routes here FIRST so dismissing the list never
    /// clears a selection built through it.
    pub fn close_candidate_popup(&mut self) -> bool {
        let was_open = self.candidate_popup.is_some();
        self.candidate_popup = None;
        self.candidate_popup_rect = None;
        was_open
    }
}

mod blit;
pub(crate) mod ecad;
mod interaction;
mod labels;
mod popups;
mod sheet;
mod touch;

pub use popups::{modal_open_last_pass, popup_open_last_pass};

/// The hit keys the viewport publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[
    crate::automation::hit_keys::HitKeyDoc { panel: "candidate", prefix: "", meaning: "one rect per entry of the open pick-candidate popup, keyed by the entry's index \u{2014} the same index `__brepCandidates` carries. `click_widget candidate/<i>` picks that entry (replace, or toggle in Click-toggles mode / with Ctrl held), which is how a script chooses between overlapping faces without a coordinate", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "pmilabel", prefix: "", meaning: "one rect per PMI label chip, keyed by annotation id \u{2014} a click SELECTS the annotation, a double click opens its form, a drag moves the label, and a RIGHT-click selects it and opens its menu (`pmimenu/menuitem:`)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "pmimenu", prefix: "menuitem:", meaning: "one rect per entry of a PMI label chip's open RIGHT-CLICK menu (pmimenu/menuitem:<annotation id>:<action id>) \u{2014} the same entries, and the same dispatch, as that annotation's PMI tree row menu (`pmi/menuitem:`): edit, move-up, move-down, delete", command: None },
    // The SHEET mode's own widgets — the third VIEWPORT-hosted widget panel,
    // in screen points with no `panel:clip` (see `viewport/sheet.rs`).
    crate::automation::hit_keys::HitKeyDoc { panel: "sheet", prefix: "paper", meaning: "the open sheet's PAPER rect in screen points \u{2014} a drag on it pans the sheet view", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "sheet", prefix: "view:", meaning: "one rect per PLACED view on the open sheet (sheet:view:<placement id>), in screen points \u{2014} DRAGGING it moves the view on the paper (one undo step per drag), a click opens its form in the Sheets pane, a RIGHT-click selects it and opens its menu (`sheet/menuitem:`)", command: Some("sheet_move_view") },
    // The paper carries NO toolbar: fit is the main toolbar's own `toolbar/fit`
    // (which frames the open sheet), zoom is the wheel, and Back to 3D and the
    // sheet's constructions are the Drawing workbench's buttons
    // (`toolbar/workbench:btn:drawing.*`), offered while a sheet is open.
    crate::automation::hit_keys::HitKeyDoc { panel: "sheet", prefix: "dim:", meaning: "one rect per PLACED dimension's value box (sheet/dim:<dimension id>) \u{2014} DRAG it to move that dimension's line, one coalesced undo step per drag; click it to SELECT the dimension, double-click it to open its form in the Sheets pane, right-click it for its menu (`sheet/menuitem:`)", command: Some("sheet_move_dimension") },
    crate::automation::hit_keys::HitKeyDoc { panel: "sheet", prefix: "ord:", meaning: "one rect per ORDINATE SET's datum value box (sheet/ord:<set id>) \u{2014} DRAG it to move the whole run's shared baseline, one coalesced undo step per drag; click it to SELECT the set, double-click it to open the set's form in the Sheets pane, right-click it for its menu (`sheet/menuitem:`). A set whose DATUM is lost publishes none: it has no datum box to grab", command: Some("sheet_move_ordinate") },
    crate::automation::hit_keys::HitKeyDoc { panel: "sheet", prefix: "menuitem:", meaning: "one rect per entry of the open RIGHT-CLICK menu on the paper (sheet/menuitem:<object id>:<action id>) \u{2014} a right-click on a placement, a dimension's value box or an ordinate set's datum box selects that object and opens its Sheets tree row menu (`sheets/menuitem:`), with the same entries and the same dispatch; a right-click on bare paper opens the open sheet's own", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "sheet", prefix: "anchor:", meaning: "one rect per ANCHOR CANDIDATE of every placement on the open sheet (sheet/anchor:<reference>, e.g. sheet/anchor:SV2:vertex:Box#1) \u{2014} each projected vertex, each named edge's two ends and midpoint, and each circular edge's centre. Published only while the REFERENCE PICKER is up for a sheet object's reference row (a dimension's anchors, an ordinate set's datum or members, a section's cutting line, a detail's centre or rim \u{2014} opened by that row's `#activate`), because the list is as long as the model's topology; clicking one adds it to the picker's list (`modebar/refsel:finish` commits). A DETAIL placement offers only the candidates inside its circle", command: Some("sheet_anchor_pick") },
    // The eCAD editors' widgets — the tile an eCAD workbench draws its editor
    // in, and the tool card over it (`viewport/ecad.rs`), in screen points.
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "canvas", meaning: "the eCAD editor's tile in screen points, while the Diagram, PCB, Symbol or Pads workbench draws its editor in place of the 3D view \u{2014} the sheet, board, symbol or pads canvas; a click, drag or wheel over it is the editor's own", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "label", meaning: "the tool card's Label name field, published while the Label tool is armed in the Diagram or PCB sheet editor; it takes the keyboard on the frame the tool arms, so the name is typed before the label is placed", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "component:", meaning: "one rect per component on the eCAD sheet (Diagram or PCB schematic), keyed by its reference (ecad/component:R1) \u{2014} the component's bounds as the editor hit-tests them; a click selects it, a drag moves it. A part whose symbol is split into GATES (a KiCad multi-unit device such as a dual op-amp) also publishes one rect per gate by the sheet's name for it (ecad/component:U1A, ecad/component:U1B), where a click selects the part with that gate picked and a drag or Rotate moves that gate alone; its bare reference is its first gate, so a drag of it moves gate A too. A Shift-click or Shift-drag on any gate picks the WHOLE part, and the drag, Move or Rotate then moves every gate together", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "pin:", meaning: "on the eCAD sheet, one small rect per component pin centred on its tip (ecad/pin:R1.2) \u{2014} where a wire starts or ends with the Wire tool, or a wiring connection is dragged from and to; in the Symbol workbench, one rect per pin of the symbol (ecad/pin:1) spanning the pin, and, for the SELECTED pin alone, one small rect on each of its two ends (ecad/pin:1:tip, ecad/pin:1:root) \u{2014} a drag from either aims that pin in one of the four directions and sets its length, holding the other end", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "unnumbered-pin:", meaning: "in the Symbol workbench, a pin whose number is BLANK, keyed by its index in the symbol (ecad/unnumbered-pin:1), spanning the pin, with `:tip` and `:root` on its two ends while it is selected, exactly as `pin:` \u{2014} out of the `pin:` family because every key under it could be some pin's number, and `pin:` bare is the family's own prefix", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "wire:", meaning: "one small rect per wire on the eCAD sheet, by its index in the document, centred on the middle of its longest segment \u{2014} a click selects the wire", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "label:", meaning: "one small rect per net label on the eCAD sheet, by index, centred on the point it labels", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "junction:", meaning: "one small rect per junction dot on the eCAD sheet, by index", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "no_connect:", meaning: "one small rect per no-connect cross on the eCAD sheet, keyed by the pin it marks (ecad/no_connect:R1.2) and centred on that pin's tip \u{2014} where a click with the No-connect tool takes the mark off", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "power_flag:", meaning: "one small rect per power flag (PWR_FLAG) on the eCAD sheet, keyed by the pin it is on (ecad/power_flag:U1.4) and centred on that pin's tip \u{2014} where a click with the Power flag tool takes the flag off", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "power_net", meaning: "the tool card's Power net field, published while the Power symbol tool is armed in the PCB sheet editor, with power_net:GND, power_net:VCC, power_net:+3V3 and power_net:+5V on the four picks above it \u{2014} the net the next power symbol placed is for", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "part:", meaning: "on the PCB board view, one rect per placed part's courtyard, keyed by its reference (ecad/part:R1) \u{2014} a click selects it, a drag moves it", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "track:", meaning: "on the PCB board view, one small rect per track by index, centred on the middle of its longest segment (a drag there slides that segment), and `track:{i}.corner:{j}` on each interior corner j of it (a drag there moves that corner)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "via:", meaning: "on the PCB board view, one rect per via by index", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "zone:", meaning: "on the PCB board view, one small rect per copper zone by index, a quarter of the way along its longest outline edge (a click selects the zone, a drag moves the whole zone and refills it), `zone:{i}.corner:{j}` on each corner j (a drag moves that corner and refills), and, while the zone is selected, `zone:{i}.edge:{j}` on the middle of each edge j (a drag there puts a new corner in)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "pad:", meaning: "one rect per PAD, its size as drawn \u{2014} in the Pads workbench keyed by the pad's number (ecad/pad:1), and on the PCB board view by its part and number (ecad/pad:R1.2), where a click SELECTS that pad in its own right and a drag from it moves the part it belongs to. A MECHANICAL pad has no number: in the Pads workbench it is `unnumbered-pad:`, and the board publishes no key for one; it is clicked by its place on the canvas", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "unnumbered-pad:", meaning: "in the Pads workbench, a MECHANICAL pad \u{2014} one whose number is BLANK \u{2014} keyed by its index in the footprint (ecad/unnumbered-pad:1), its size as drawn, exactly as `pad:` \u{2014} out of the `pad:` family because every key under it could be some pad's number, and `pad:` bare is the family's own prefix. Its Inspector row is `inspector:row:unnumbered-pad:<index>`", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "silk:", meaning: "in the Pads workbench, one small rect per SILKSCREEN line by its index in the footprint (ecad/silk:0), centred on the middle of the line's longest segment \u{2014} a click selects that line (a pad under the same point wins, as it does for a pointer), a drag from it moves it. In the PCB board view, one per silkscreen line of each placed part, by the part's reference and the line's index in its footprint (ecad/silk:R1.0), on a point the canvas answers with that line \u{2014} a click selects the line (copper under the same point wins), a drag from it moves the PART, and Delete leaves it, since it belongs to the footprint", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "inspector:", meaning: "in the Symbol, Pads and PCB (board view) workbenches, the INSPECTOR's buttons, fields and list rows (ecad/inspector:next_unbound, ecad/inspector:number_it, ecad/inspector:pin_number, ecad/inspector:aim:up, ecad/inspector:tip:x; Pads: ecad/inspector:place_pad:<pin>, ecad/inspector:put_on:<pin>, ecad/inspector:make_mechanical, ecad/inspector:add_copies; PCB: ecad/inspector:autoroute, ecad/inspector:vias and its choices ecad/inspector:vias:allow|avoid|never, ecad/inspector:run_drc, a finding ecad/inspector:row:finding:<index in the list>, ecad/inspector:fit_outline, ecad/inspector:auto_place, ecad/inspector:outline_width, ecad/inspector:apply_size, ecad/inspector:copper_layers, ecad/inspector:rules_section, ecad/inspector:rule:clearance, ecad/inspector:apply_rules, the Net classes section ecad/inspector:net_classes_section, its net picker ecad/inspector:net_class:net and a net in it ecad/inspector:net_class:net:+5V, the class a net is put in ecad/inspector:net_class:assign with ecad/inspector:net_class:assign:auto (back to the patterns) and ecad/inspector:net_class:assign:Power, the net's readout ecad/inspector:net_class:readout, ecad/inspector:net_class:add, and per class ecad/inspector:net_class:Default and ecad/inspector:net_class:Power:name|clearance|track_width|via_diameter|via_drill|patterns|delete, for a selected zone ecad/inspector:zone_net, ecad/inspector:zone_layer, ecad/inspector:thermal_gap, ecad/inspector:spoke_width, ecad/inspector:min_width, ecad/inspector:fill_zones and with the Zone tool up ecad/inspector:zone_net, ecad/inspector:close_zone, and for a selected part ecad/inspector:rotate, ecad/inspector:flip, ecad/inspector:row:pad:R1.2), a list row by the key its item has on the canvas (ecad/inspector:row:pin:3, ecad/inspector:row:silk:0), and each list's resize grip (ecad/inspector:items_grip; Pads also ecad/inspector:pins_grip, under its Symbol pins list); on the Diagram and PCB SHEET, the Connectivity panel's electrical rule check: its button (ecad/inspector:erc:run, which runs the check and, pressed again, hides the findings), one row per finding by its place in the list (ecad/inspector:erc:row:0, a click zooms the sheet to it) and the fix a finding offers under its row (ecad/inspector:erc:fix:0) \u{2014} each where a click presses it, and only while at least half of it is in view and the Inspector is drawn: a widget scrolled out of the pane or out of a list is not published, and nor is anything while the Inspector is behind another tab", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "menuitem:", meaning: "on the PCB board view, one rect per entry of the canvas's open RIGHT-CLICK menu (ecad/menuitem:rotate, ecad/menuitem:flip_side, ecad/menuitem:select_part, ecad/menuitem:route_from_here, ecad/menuitem:fill_zones, ecad/menuitem:delete) \u{2014} a right-click on a part, pad or silkscreen line selects it and opens its part's menu, on a track or a via its own; published only while the menu is open", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "stock", meaning: "the tool card's Stock part for new connections field, published while a wiring diagram's Select tool is up; each connection drawn after it carries that stock part number", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "outdated:card", meaning: "the out-of-date card over the top middle of the Diagram or PCB tile (folded to a one-line chip after Later), published while a component on the sheet was placed from a part whose saved version changed since (ecad_state's outdated lists them)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "outdated:later", meaning: "the out-of-date card's Later button: folds the card to a one-line chip (outdated:card, a click opens it again) and changes nothing; the components stay ringed, and a changed list opens the card again", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "unreadable:card", meaning: "the card that takes the whole Diagram, PCB, Symbol or Pads tile while the editor could not read its block (ecad_state's unreadable says why): it names the parts at fault and never shows the empty sheet's first-run card", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "unreadable:fix", meaning: "the unreadable card's repair (ecad_state's repair, e.g. Rename the second R5 to R6): every repeated or blank reference to the next free one of its prefix, as one undo step; drawn only where a rename frees the sheet", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecad", prefix: "outdated:update", meaning: "the out-of-date card's Update parts button: Assembly > Constraints > Update components, run from where the user is, as one undo step", command: None },
    // A VIEWPORT-hosted widget, so its rects are in screen points and the panel
    // publishes no `panel:clip` (the viewport does not scroll) — see
    // `Viewport::gizmo_anchor_hits_json`, which is why this is not in the
    // history panel's blob.
    crate::automation::hit_keys::HitKeyDoc { panel: "gizmo", prefix: "anchor", meaning: "the armed gizmo's FRAME ORIGIN as a zero-size rect in SCREEN points — in transform mode the orange CENTRE free-move handle, which a drag free-moves the frame in the view plane from and a click toggles to the dimension gizmo. In DIMENSION mode it is still the frame origin, which is where the ◎ origin ball is drawn only when the feature's pivot and its leaders' anchor coincide (a cube at the world origin): the balls themselves are published as `gizmo/ball:`, and that is what a script should click to toggle back", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "gizmo", prefix: "axis:", meaning: "one zero-size rect per AXIS ARROW of the drawn transform gizmo (gizmo/axis:x|y|z, the frame's own axes), in SCREEN points, three quarters of the way along the arrow \u{2014} a drag from it translates the frame along that axis, which writes the feature's `transform.position`. Published only where the gizmo's own pick answers with that arrow: an arrow pointing at the camera has none", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "gizmo", prefix: "ring:", meaning: "one zero-size rect per ROTATION GRAB ball of the drawn transform gizmo (gizmo/ring:x|y|z, named by the frame axis its arc turns about), in SCREEN points \u{2014} a drag from it rotates the frame about that axis, through the gizmo's origin, which writes the feature's `transform.rotationEuler`. Published only where the gizmo's own pick answers with that ring: an arc seen edge-on, its ball lying on an arrow, has none", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "gizmo", prefix: "dim:", meaning: "one zero-size rect per grabbable ARROWHEAD of the drawn \u{25ce} DIMENSION gizmo (gizmo/dim:<field key> \u{2014} gizmo/dim:sizeX, gizmo/dim:radius, gizmo/dim:distance), in SCREEN points, at the cone TIP of a linear leader or the sweep-END ball of an angular one \u{2014} a DRAG from it edits that parameter live, exactly as typing into the form's `history/field:<the same key>` does, and the geometry rebuilds as the pointer travels. Published only where the gizmo's own pick answers with that field, and only while the DIMENSION gizmo is armed \u{2014} opening a feature's form arms it", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "gizmo", prefix: "ball:", meaning: "one zero-size rect per orange ORIGIN BALL of the drawn ◎ DIMENSION gizmo (gizmo/ball:<field key>), in SCREEN points — the sphere every leader anchored there starts from, keyed by the first of them, so a cube's three dims share gizmo/ball:sizeX while a cone draws gizmo/ball:radiusBottom and gizmo/ball:radiusTop. A CLICK on one TOGGLES the gizmo back to its transform form, which is the only route an angular-only feature (a revolve) has to the move handles; the angle's own sweep-end ball is a VALUE handle and is published as `gizmo/dim:` instead. Published only where the gizmo's own pick answers with that ball", command: None },
    // The DRAGGABLE constraint handles — the second VIEWPORT-hosted widget
    // panel, in screen points with no `panel:clip`, keyed by constraint id (see
    // `constraint_handle_hits_json`).
    crate::automation::hit_keys::HitKeyDoc { panel: "constraint", prefix: "", meaning: "one zero-size rect per DRAGGABLE assembly-constraint handle, keyed by constraint id \u{2014} a distance arrow's tip, an angle arc's sweep-end handle, in SCREEN points. A bare click is swallowed; DRAGGING it edits that constraint's value and re-solves, which is what `assembly_update_constraint` does without a pointer", command: Some("assembly_update_constraint") },
];
