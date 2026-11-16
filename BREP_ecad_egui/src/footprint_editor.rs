//! Footprint editor: pads, holes, and silkscreen, working like the symbol editor.
use super::*;
use crate::editor_core::EditorCore;
use crate::actions::{Action, ActionGroup, action, command, key};
use crate::board_view::{BACKGROUND, SILK, THROUGH_HOLE, WARNING, layer_color};
use crate::symbol_editor::{InspectorHits, LIST_SHARE, grip_under, key_word, list_height};
use brep_ecad_core::board::{COURTYARD_MARGIN, DesignRules, Footprint, Pad, PadShape, PadVias, PasteWindows, Shape};
use brep_ecad_core::graphic::{GraphicLayer, GraphicShape, LayerGraphic};
use crate::board_view::graphic_color;

#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum DrawTool {
    Select,
    Pad,
    ThroughHole,
    Line,
    Rectangle,
    Circle,
    Arc,
    Text,
}
#[derive(Clone, Copy, PartialEq, Debug)]
enum Item {
    Pad(usize),
    Silk(usize),
    /// One of the footprint's drawn items on a non-copper layer, by its index
    /// in `Footprint::graphics`.
    Graphic(usize),
}
#[derive(Clone, Copy, PartialEq)]
enum PadKind {
    SurfaceMount,
    ThroughHole,
    Unplated,
}
/// What a drawn item is linked to.
///
/// A footprint carries two kinds of thing and only ONE of them is a conductor. A
/// PAD is copper: it reaches a symbol pin by carrying that pin's number, which is
/// the same name the pin and the part's connection point carry, and that number
/// IS the link — there is nothing else to bind. A silkscreen line is legend: it
/// prints on the silkscreen film ([`brep_ecad_core::fabrication`] draws it there
/// and nowhere else) and never enters the netlist, so it has no pin to be linked
/// to and none is invented for it. What a user can do with a drawn shape is turn
/// it INTO a pad ([`FootprintEditor::silk_as_pad`]), which is a different thing
/// from linking it.
#[derive(Clone, Copy, PartialEq, Debug)]
enum PadLink {
    /// The number is a pin of the symbol these pads belong to.
    Pin,
    /// A number no pin carries: the symbol has not got that pin, or the number is
    /// a slip. The one state worth a warning.
    NoSuchPin,
    /// No number: a MECHANICAL pad, deliberately on no net — a mounting hole, a
    /// shield tab, a castellation. Not an error, and never shown as one.
    Mechanical,
    /// Nothing here knows what pins there are, so nothing judges the number: a
    /// footprint edited on its own, before any symbol stands beside it.
    Unknown,
}
/// Which pin, if any, the number `number` names, given the symbol's pin numbers
/// and whether they are known at all. A free function because the properties
/// panel asks it while it holds the pad it is editing.
fn link_of(number: &str, pins: &[String], known: bool) -> PadLink {
    if number.is_empty() {
        PadLink::Mechanical
    } else if !known {
        PadLink::Unknown
    } else if pins.iter().any(|pin| pin == number) {
        PadLink::Pin
    } else {
        PadLink::NoSuchPin
    }
}
/// What the link means, in the words shown beside the pad it is about.
fn link_note(link: PadLink) -> &'static str {
    match link {
        PadLink::Pin => "On the symbol pin of this number.",
        PadLink::NoSuchPin => {
            "No pin of the symbol carries this number, so this pad is on no net. Choose a pin below, or make it mechanical."
        }
        PadLink::Mechanical => {
            "Mechanical: no pin and no net. It reaches the copper and drill files and never the netlist."
        }
        PadLink::Unknown => "No symbol pins are known here, so nothing checks this number.",
    }
}
/// How far a pad's link ring stands clear of the pad, and how wide it is, in
/// points: the gap is the selection outline's width, so a selected pad shows
/// both, the outline inside the ring.
const RING_GAP: f32 = 2.;
const RING_WIDTH: f32 = 2.;
/// The ring a pad wears on the canvas for each link state — nothing for a pad
/// that is on its pin, which is the ordinary case and needs no mark.
fn link_ring(link: PadLink) -> Option<Color32> {
    match link {
        PadLink::Pin | PadLink::Unknown => None,
        PadLink::Mechanical => Some(Color32::from_gray(132)),
        PadLink::NoSuchPin => Some(WARNING),
    }
}

/// What the Inspector shows beside a word to say which mark on the canvas the
/// word is about: a pad in its ring, or a silkscreen line.
#[derive(Clone, Copy)]
enum Mark {
    Ring(Color32),
    Silk,
    /// A drawn item, in its layer's colour.
    Drawn(Color32),
}
/// A mark as the canvas draws it — its colours, on the canvas's own dark
/// ground — in a chip before the words about it. The WORDS are drawn in the
/// theme's colours: the canvas's colours are chosen for its dark ground, and
/// on the light theme's panel its orange measured 2.64:1 and its silkscreen
/// white 1.22:1, under the 2.77:1 of the theme's own weakest text. The chip
/// keeps what the colours were there for, that the word and the mark are
/// recognisably one. `None` leaves the chip's room empty, so a list's words
/// stay in one column.
fn swatch(ui: &mut egui::Ui, mark: Option<Mark>) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(22., 12.), Sense::hover());
    let Some(mark) = mark else { return };
    let painter = ui.painter();
    painter.rect_filled(rect, 2., BACKGROUND);
    match mark {
        // The canvas's own mark: the ring clear of the pad, on the dark ground.
        Mark::Ring(colour) => {
            let pad = Rect::from_center_size(rect.center(), Vec2::new(10., 4.));
            painter.rect_filled(pad, 0., layer_color(0, 2));
            painter.rect_stroke(
                pad.expand(RING_GAP),
                0.,
                Stroke::new(RING_WIDTH, colour),
                egui::StrokeKind::Outside,
            );
        }
        Mark::Silk => {
            painter.line_segment(
                [rect.left_center() + Vec2::new(4., 0.), rect.right_center() - Vec2::new(4., 0.)],
                Stroke::new(1.5, SILK),
            );
        }
        Mark::Drawn(colour) => {
            painter.line_segment(
                [rect.left_center() + Vec2::new(4., 0.), rect.right_center() - Vec2::new(4., 0.)],
                Stroke::new(2., colour),
            );
        }
    }
}

/// Snapping steps offered in the toolbar, the Inspector and as `pads.grid.*`,
/// in micrometres: five metric, and the three imperial pitches parts come in
/// (25, 50 and 100 mil). A 50 µm grid put pads meant 1.27 mm apart at 1.25.
const GRIDS: [i32; 8] = [10, 50, 100, 250, 500, 635, 1270, 2540];
/// A grid step as a menu names it: millimetres, and mils for the imperial ones.
fn grid_text(grid: i32) -> String {
    match grid {
        635 => "0.635 mm (25 mil)".into(),
        1270 => "1.27 mm (50 mil)".into(),
        2540 => "2.54 mm (100 mil)".into(),
        _ => format!("{} mm", mm_text(grid)),
    }
}
/// Pad sizes used when a pad is placed with a click instead of a drag.
const SMD_SIZE: Point = Point::new(1500, 900);
const THT_SIZE: Point = Point::new(1700, 1700);
/// The Items list's height in a short pane: the 200 points it had before it
/// could be sized. A tall pane gives it [`LIST_SHARE`] of its height, as the
/// Symbol Inspector gives its Items list.
const ITEMS_HEIGHT: f32 = 200.;
/// The Symbol pins list's height in a short pane, the 150 points it had; a
/// tall pane gives it [`PINS_SHARE`].
const PINS_HEIGHT: f32 = 150.;
/// Less than the Items list's share: the two lists sit one above the other,
/// with the selected item's fields under both.
const PINS_SHARE: f32 = 0.2;

/// How many pins with no pad are offered as BUTTONS beside a pad's number. Past
/// that they are a dropdown: a two-pin part's choice should be one click, and a
/// forty-pin part's should not be forty buttons.
const PIN_BUTTONS: usize = 8;

/// Draws one footprint: its pads, holes, and silkscreen. The host places
/// [`Self::toolbar`], [`Self::properties`], and [`Self::show`], runs commands from
/// [`pad_actions`] by id, and takes each edit with [`Self::take_change`].
pub struct FootprintEditor {
    pub footprint: Footprint,
    /// The part this footprint came from; Apply updates it.
    pub target: Option<Uuid>,
    /// Pin numbers of the target's symbol, to show pins that still lack a pad.
    pins: Vec<String>,
    /// The symbol's pin names by number, for the Symbol pins list: a number alone
    /// says little about which pin a pad is for. Empty when the host has not said.
    pin_names: std::collections::BTreeMap<String, String>,
    core: EditorCore<Footprint, Item, (Footprint, Point)>,
    tool: DrawTool,
    /// The layer and pen the drawing tools draw with; no pen is the layer's own.
    draw_layer: GraphicLayer,
    draw_width: Option<i32>,
    start: Option<Point>,
    fit: bool,
    grid: i32,
    /// Copies, and step between them, for repeating the selected pad.
    repeat: (u32, i32, i32),
    /// The pin number the NEXT pad placed will carry, chosen before the pad
    /// exists: the Symbol pins list's "Place pad" arms it, so a pad placed for a
    /// pin is never unlinked, not even for the frame between the click and the
    /// number.
    pending_number: Option<String>,
    /// The canvas the last frame drew on, in screen points: what [`Self::canvas`]
    /// was given. Held so a selection a HOST asks for can be brought into view.
    canvas: Rect,
    error: String,
    /// The Inspector's clickable widgets as it last drew them, for [`Self::hits`].
    inspector: InspectorHits,
    toolbar_hits: InspectorHits,
    /// The pass the canvas was last drawn in: the Inspector's rects are
    /// published only while it is drawn beside it.
    canvas_pass: u64,
    /// The selection the lists last showed: one made since, on the canvas or
    /// by a host, scrolls its row into view.
    listed: Option<Item>,
    /// The heights the user dragged the Items and Symbol pins lists to with
    /// their grips, or `None` for the pane's share.
    items_height: Option<f32>,
    pins_height: Option<f32>,
}
impl FootprintEditor {
    pub fn new(footprint: Footprint, target: Option<Uuid>, pins: Vec<String>) -> Self {
        Self {
            footprint,
            target,
            pins,
            pin_names: Default::default(),
            core: EditorCore::new(0.08),
            tool: DrawTool::Select,
            draw_layer: GraphicLayer::FrontSilk,
            draw_width: None,
            start: None,
            fit: true,
            grid: 50,
            repeat: (1, 0, 1270),
            pending_number: None,
            canvas: Rect::from_min_size(Pos2::ZERO, Vec2::new(800., 600.)),
            error: String::new(),
            inspector: InspectorHits::default(),
            toolbar_hits: InspectorHits::default(),
            canvas_pass: 0,
            listed: None,
            items_height: None,
            pins_height: None,
        }
    }
    pub fn blank() -> Self {
        Self::new(
            Footprint {
                name: "Custom:NewFootprint".into(),
                pads: vec![],
                silk: vec![],
                model: None,
                courtyard_margin: None,
                graphics: vec![],
            },
            None,
            vec![],
        )
    }
    fn record(&mut self, before: Footprint) {
        self.record_as(before, None);
    }
    /// Record a finished edit. Edits sharing `coalesce` are reported to the host as one
    /// change; the properties panel uses one key for all of its fields, so consecutive
    /// edits there merge whichever field they touched.
    fn record_as(&mut self, before: Footprint, coalesce: Option<&str>) {
        self.core.record(before, &self.footprint, coalesce);
    }
    /// The edits made since the last call, merged into one. A host that keeps the
    /// footprint in its own file calls this once a frame and stores [`Self::footprint`].
    pub fn take_change(&mut self) -> Option<Change> {
        self.core.take_change()
    }
    /// The drawing tool that is up, by the last word of the action that picks it
    /// (`pads.tool.<name>`). Read-only, for a host that publishes it.
    pub fn tool_name(&self) -> &'static str {
        match self.tool {
            DrawTool::Select => "select",
            DrawTool::Pad => "smd",
            DrawTool::ThroughHole => "through_hole",
            DrawTool::Line => "line",
            DrawTool::Rectangle => "rectangle",
            DrawTool::Circle => "circle",
            DrawTool::Arc => "arc",
            DrawTool::Text => "text",
        }
    }
    /// Where the canvas is looking: the footprint origin's offset from the canvas centre in
    /// points, and the zoom in points per micrometre. Read-only, for a host that shows it.
    pub fn pan_zoom(&self) -> (Vec2, f32) {
        (self.core.pan, self.core.zoom)
    }
    /// The pin numbers of the symbol these pads belong to, so the editor lists the
    /// pins that still have no pad. For a host that keeps the symbol beside the
    /// pads, as a part's two blocks, rather than opening the editor on a
    /// component; it is not an edit.
    pub fn set_pins(&mut self, pins: Vec<String>) {
        self.pins = pins;
    }
    /// The symbol's pins as `(number, name)`, for a host that has their names as
    /// well: the Symbol pins list then reads `7 GND`, not `7`. Sets the pin
    /// numbers too, exactly as [`Self::set_pins`] does. An unnamed pin (empty or
    /// KiCad's `~`) and one named by its own number read as the number alone.
    pub fn set_pin_names(&mut self, pins: Vec<(String, String)>) {
        self.pins = pins.iter().map(|(number, _)| number.clone()).collect();
        self.pin_names = pins
            .into_iter()
            .filter_map(|(number, name)| {
                let name = crate::pin_name(&name)?.to_owned();
                (name != number).then_some((number, name))
            })
            .collect();
    }
    /// How the Symbol pins list names pin `number`: `7 GND`, or `7` when it has
    /// no name of its own.
    fn pin_row(&self, number: &str) -> String {
        match self.pin_names.get(number) {
            Some(name) => format!("{number} {name}"),
            None => number.to_owned(),
        }
    }
    /// The NUMBER of the selected pad, or `None` when the selection is silk or
    /// empty. The symbol-wide identity a host joins on, exactly as
    /// [`SymbolEditor::selected_pin`] is; `Item` stays private.
    pub fn selected_pad(&self) -> Option<&str> {
        match self.core.selected {
            Some(Item::Pad(index)) => {
                self.footprint.pads.get(index).map(|pad| pad.number.as_str())
            }
            _ => None,
        }
    }
    /// Select the pad numbered `number`, and say whether there was one. The
    /// [`SymbolEditor::select_pin`] twin: a host jumps here from a pin or a
    /// connection point of the same name. Not an edit.
    pub fn select_pad(&mut self, number: &str) -> bool {
        match self.footprint.pads.iter().position(|pad| pad.number == number) {
            Some(index) => {
                self.core.selected = Some(Item::Pad(index));
                // And bring it into view if it is off the canvas. A host's jump
                // from a connection point has to SHOW the pad, and the canvas may
                // be zoomed in on another corner of the footprint; a pad already on
                // screen does not move it.
                let at = self.footprint.pads[index].at;
                let centre = self.canvas.center() + self.core.pan;
                if !self
                    .canvas
                    .shrink(12.)
                    .contains(on_canvas(centre, self.core.zoom, at))
                {
                    self.core.pan = -Vec2::new(at.x as f32, at.y as f32) * self.core.zoom;
                }
                true
            }
            None => false,
        }
    }
    /// Where each pad is drawn, `pad:<number>` (`pad_key`: a mechanical
    /// pad's is `unnumbered-pad:<index>`), and each silkscreen line,
    /// `silk:<index>` (a small rect on the middle of its longest segment, as a
    /// sheet wire's is), in screen points on the `canvas` [`Self::show`] was
    /// given — the mapping the canvas paints with — and then the Inspector's
    /// widgets, `inspector:<thing>`, where [`Self::properties`] last drew them.
    pub fn hits(&self, canvas: Rect) -> Vec<(String, Rect)> {
        let center = canvas.center() + self.core.pan;
        let screen = |p: Point| on_canvas(center, self.core.zoom, p);
        let pads = self.footprint.pads.iter().enumerate().map(|(i, pad)| {
            let size = Vec2::new(pad.size.x.abs() as f32, pad.size.y.abs() as f32) * self.core.zoom;
            let rect = Rect::from_center_size(screen(pad.at), size.max(Vec2::splat(6.)));
            (pad_key(i, pad), rect)
        });
        let silk = self.footprint.silk.iter().enumerate().filter_map(|(i, line)| {
            let (a, b) = line
                .windows(2)
                .map(|s| (screen(s[0]), screen(s[1])))
                .max_by(|a, b| a.0.distance(a.1).total_cmp(&b.0.distance(b.1)))?;
            Some((format!("silk:{i}"), Rect::from_center_size(a.lerp(b, 0.5), Vec2::splat(8.))))
        });
        // A drawn item the same way, `graphic:<index>`, text by its anchor: on
        // the middle of its longest segment the canvas answers with THIS item
        // (a pad or a line over a middle takes the click there), else the
        // longest.
        let world = |p: Pos2| {
            let v = (p - center) / self.core.zoom;
            Point::new(v.x.round() as i32, v.y.round() as i32)
        };
        let graphics = self.footprint.graphics.iter().enumerate().filter_map(|(i, g)| {
            let at = match &g.shape {
                // Text by the middle of the box its strokes fill, which is
                // where it answers a click.
                GraphicShape::Text { at, .. } => {
                    let points: Vec<Pos2> = g.strokes(!g.layer.front()).iter().flatten().map(|p| screen(*p)).collect();
                    if points.is_empty() { screen(*at) } else { Rect::from_points(&points).center() }
                }
                _ => {
                    let mut middles: Vec<(f32, Pos2)> = g
                        .strokes(!g.layer.front())
                        .iter()
                        .flat_map(|line| line.windows(2).map(|s| (screen(s[0]), screen(s[1]))).collect::<Vec<_>>())
                        .map(|(a, b)| (a.distance(b), a.lerp(b, 0.5)))
                        .collect();
                    middles.sort_by(|a, b| b.0.total_cmp(&a.0));
                    let longest = middles.first()?.1;
                    middles
                        .iter()
                        .map(|m| m.1)
                        .find(|&m| self.hit(m, screen, world(m)) == Some(Item::Graphic(i)))
                        .unwrap_or(longest)
                }
            };
            Some((format!("graphic:{i}"), Rect::from_center_size(at, Vec2::splat(8.))))
        });
        pads.chain(silk).chain(graphics).chain(self.inspector.shown(self.canvas_pass))
            .chain(self.toolbar_hits.shown(self.canvas_pass)).collect()
    }
    /// The index of the selected silkscreen line, or `None` when the selection
    /// is a pad or empty — [`Self::selected_pad`]'s twin for silk, which has
    /// no number and is named by its place in the list, as `silk:<index>` is.
    pub fn selected_silk(&self) -> Option<usize> {
        match self.core.selected {
            Some(Item::Silk(index)) if index < self.footprint.silk.len() => Some(index),
            _ => None,
        }
    }
    /// The index of the selected drawn item in `Footprint::graphics`, as
    /// `graphic:<index>` names it, or `None`.
    pub fn selected_graphic(&self) -> Option<usize> {
        match self.core.selected {
            Some(Item::Graphic(index)) if index < self.footprint.graphics.len() => Some(index),
            _ => None,
        }
    }
    /// Load a footprint the host holds, keeping zoom, pan, and grid. The editor's own
    /// undo history starts afresh and the load is not reported by [`Self::take_change`].
    pub fn set_footprint(&mut self, footprint: Footprint) {
        self.footprint = footprint;
        self.core.history = History::default();
        self.core.selected = None;
        self.core.drag = None;
        self.start = None;
        self.error.clear();
        self.core.pending_change = None;
    }
    /// Run a command from [`pad_actions`] by id, as a click on its button does.
    pub fn run_action(&mut self, id: &str) -> bool {
        crate::actions::run_action(self, pad_actions().iter(), id)
    }
    fn set_tool(&mut self, tool: DrawTool) {
        if let Some((before, _)) = self.core.drag.take() {
            self.footprint = before;
        }
        // A pin waiting for a pad survives a change between the two pad tools —
        // through-hole or surface mount is still that pin's pad — and is put down
        // with anything else, Select and Escape included.
        if !matches!(tool, DrawTool::Pad | DrawTool::ThroughHole) {
            self.pending_number = None;
        }
        self.tool = tool;
        self.start = None;
    }
    fn undo(&mut self) {
        self.core.undo(&mut self.footprint);
    }
    pub(crate) fn can_step_history(&self, key: HistoryKey) -> bool {
        self.core.can_step_history(key)
    }
    fn redo(&mut self) {
        self.core.redo(&mut self.footprint);
    }

    /// Whether the symbol's pins are known here at all: a part is open, or the
    /// host has named some. A part whose symbol has NO pins is known and empty,
    /// so a numbered pad on it is honestly reported as being on no pin.
    fn pins_known(&self) -> bool {
        self.target.is_some() || !self.pins.is_empty()
    }
    /// Which pin one pad is on, if any.
    fn link(&self, pad: &Pad) -> PadLink {
        link_of(&pad.number, &self.pins, self.pins_known())
    }
    /// How each pad reads: `(on a pin, mechanical, on no pin)`.
    fn link_counts(&self) -> (usize, usize, usize) {
        let mut counts = (0, 0, 0);
        for pad in &self.footprint.pads {
            match self.link(pad) {
                PadLink::Pin | PadLink::Unknown => counts.0 += 1,
                PadLink::Mechanical => counts.1 += 1,
                PadLink::NoSuchPin => counts.2 += 1,
            }
        }
        counts
    }
    /// The numbers carried by pads that no pin of the symbol has.
    fn orphan_numbers(&self) -> Vec<&str> {
        self.footprint
            .pads
            .iter()
            .filter(|pad| self.link(pad) == PadLink::NoSuchPin)
            .map(|pad| pad.number.as_str())
            .collect()
    }
    /// How one item reads in a list, and on hover: what it is, and what it is on.
    fn item_note(&self, item: Item) -> String {
        match item {
            Item::Pad(i) => match self.footprint.pads.get(i) {
                None => "A pad that is no longer here".to_owned(),
                Some(pad) => {
                    let kind = match (pad.drill, pad.plated) {
                        (None, _) => "Pad",
                        (Some(_), true) => "Through-hole pad",
                        (Some(_), false) => "Unplated hole",
                    };
                    match self.link(pad) {
                        PadLink::Mechanical => format!("{kind} · mechanical"),
                        PadLink::NoSuchPin => format!("{kind} {} · no pin {}", pad.number, pad.number),
                        PadLink::Pin | PadLink::Unknown => format!("{kind} {}", pad.number),
                    }
                }
            },
            Item::Silk(i) => format!("Silkscreen {} · decoration", i + 1),
            Item::Graphic(i) => match self.footprint.graphics.get(i) {
                Some(g) => format!("{} {} · {}", graphic_kind(g), i + 1, g.layer.name()),
                None => format!("Drawing {}", i + 1),
            },
        }
    }
    /// Arm a pad tool to place the next pad as pin `number`'s. The other half of
    /// linking by clicking: the number is chosen first and the click says WHERE,
    /// which is the gesture a pad is always placed with.
    fn place_pad_for(&mut self, number: String) {
        if !matches!(self.tool, DrawTool::Pad | DrawTool::ThroughHole) {
            self.tool = DrawTool::Pad;
        }
        self.start = None;
        self.core.drag = None;
        self.pending_number = Some(number);
    }
    /// Symbol pins with no pad of the same number.
    fn missing_pins(&self) -> Vec<&str> {
        self.pins
            .iter()
            .filter(|pin| !self.footprint.pads.iter().any(|p| &p.number == *pin))
            .map(String::as_str)
            .collect()
    }
    /// The lowest unused positive pad number.
    fn next_number(&self) -> String {
        (1..)
            .map(|n: u32| n.to_string())
            .find(|n| !self.footprint.pads.iter().any(|p| p.number == *n))
            .unwrap_or_default()
    }
    fn snap(&self, p: Point) -> Point {
        let g = f64::from(self.grid);
        Point::new(
            ((f64::from(p.x) / g).round() * g) as i32,
            ((f64::from(p.y) / g).round() * g) as i32,
        )
    }
    /// The snapping grid, in micrometres.
    pub fn grid(&self) -> i32 {
        self.grid
    }
    /// Snap to `grid` micrometres from now on, when it is one of [`GRIDS`].
    fn set_grid(&mut self, grid: i32) {
        if GRIDS.contains(&grid) {
            self.grid = grid;
        }
    }
    fn grid_combo(&mut self, ui: &mut egui::Ui, salt: &str) -> egui::Response {
        let ids: Vec<_> = GRIDS.iter().map(i32::to_string).collect();
        let labels: Vec<_> = GRIDS.iter().map(|&grid| grid_text(grid)).collect();
        let choices: Vec<_> = GRIDS.iter().zip(&ids).zip(&labels)
            .map(|((&grid, id), label)| (grid, id.as_str(), label.as_str())).collect();
        ui.push_id(salt, |ui| {
            if salt == "footprint-grid" {
                self.toolbar_hits.begin(ui.ctx());
                self.toolbar_hits.enum_dropdown(ui, "toolbar_grid", &choices, &mut self.grid)
            } else {
                self.inspector.enum_dropdown(ui, "grid", &choices, &mut self.grid)
            }
        }).inner
    }
    /// Pairs of copper pads closer than [`min_pad_gap`], with their gap in
    /// micrometres, closest first. Two pads of ONE number are one pin (a
    /// connector's repeated tab), so they may touch; an unplated hole is no
    /// copper. What DRC would find on the board, found where it is made.
    fn close_pads(&self) -> Vec<(usize, usize, f64)> {
        let minimum = f64::from(min_pad_gap());
        let pads = &self.footprint.pads;
        let copper = |pad: &Pad| pad.drill.is_none() || pad.plated;
        let mut close = vec![];
        for (i, a) in pads.iter().enumerate().filter(|(_, p)| copper(p)) {
            for (j, b) in pads.iter().enumerate().skip(i + 1).filter(|(_, p)| copper(p)) {
                if !a.number.is_empty() && a.number == b.number {
                    continue;
                }
                // Copper on opposite faces of the board never meets.
                let (af, ab) = a.copper_sides();
                let (bf, bb) = b.copper_sides();
                if !(af && bf) && !(ab && bb) {
                    continue;
                }
                let gap = a.local_shape().distance(&b.local_shape());
                if gap + 0.5 < minimum {
                    close.push((i, j, gap));
                }
            }
        }
        close.sort_by(|a, b| a.2.total_cmp(&b.2));
        close
    }
    /// The tool buttons, history, zoom, and grid, with a line on the tool in use.
    pub fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            // The tools split the way the footprint does: COPPER, which carries a
            // pin, and SILKSCREEN, which carries nothing. Which of the two a tool
            // draws is the first thing to know about it, so the row says it rather
            // than leaving six buttons in a line to be told apart by name.
            crate::actions::action_buttons(
                self,
                pad_actions().iter().filter(|a| !SILK_TOOLS.contains(&a.id)),
                ui,
                ActionGroup::Tool,
            );
            ui.separator();
            ui.small("Silkscreen");
            crate::actions::action_buttons(
                self,
                pad_actions().iter().filter(|a| SILK_TOOLS.contains(&a.id)),
                ui,
                ActionGroup::Tool,
            );
            ui.separator();
            for group in [ActionGroup::History, ActionGroup::Zoom] {
                crate::actions::action_buttons(self, pad_actions().iter(), ui, group);
            }
            self.grid_combo(ui, "footprint-grid");
        });
        ui.label(match (self.pending_number.clone(), self.tool) {
            (Some(pin), _) => format!(
                "Click to place the pad for pin {pin}; it is on that pin from the moment it exists. Select puts the tool down."
            ),
            (None, DrawTool::Select) => {
                "Select or drag an item. Right-drag pans. Scroll zooms. Pads are viewed from the top.".to_owned()
            }
            (None, DrawTool::Pad | DrawTool::ThroughHole) => {
                "Click to place a pad, or drag to draw its outline. A new pad takes the lowest free number; Item properties puts it on a pin.".to_owned()
            }
            _ => {
                "Drag to draw silkscreen — decoration, on no pin and no net. Item properties can turn a rectangle or a circle into a pad.".to_owned()
            }
        });
    }
    /// What the canvas's marks mean — and only the marks it is actually making. A
    /// legend for states this footprint has not got is noise, so each line appears
    /// with the thing it explains.
    fn legend(&self, ui: &mut egui::Ui) {
        let (_, mechanical, orphans) = self.link_counts();
        if mechanical == 0 && orphans == 0 && self.footprint.silk.is_empty() {
            return;
        }
        ui.horizontal_wrapped(|ui| {
            ui.small("On the canvas:");
            // Each line its mark as the canvas draws it, then its words in
            // the theme's colours ([`swatch`]).
            if let Some(ring) = link_ring(PadLink::Mechanical).filter(|_| mechanical > 0) {
                swatch(ui, Some(Mark::Ring(ring)));
                ui.small("grey ring — mechanical, on no pin");
            }
            if let Some(ring) = link_ring(PadLink::NoSuchPin).filter(|_| orphans > 0) {
                swatch(ui, Some(Mark::Ring(ring)));
                ui.label(
                    egui::RichText::new("orange ring — no pin of that number")
                        .small()
                        .color(ui.visuals().warn_fg_color),
                );
            }
            if !self.footprint.silk.is_empty() {
                swatch(ui, Some(Mark::Silk));
                ui.small("thin line — silkscreen, decoration");
            }
        });
    }
    /// The drawing canvas, filling the space it is given.
    pub fn show(&mut self, ui: &mut egui::Ui) {
        let size = ui.available_size();
        self.canvas(ui, size);
    }
    /// Run the commands whose keys were pressed, unless a field is taking the
    /// keys or a menu is open: [`crate::keys_reach_editor`], the rule the Diagram
    /// and PCB editors apply. The host runs this BEFORE the canvas draws its
    /// right-click menu, so without the menu clause Escape was spent on Select
    /// and the menu, which closes on that key, stayed up.
    pub fn run_shortcuts(&mut self, ctx: &egui::Context) {
        if crate::keys_reach_editor(ctx) {
            let actions: Vec<_> = pad_actions().iter().collect();
            crate::actions::run_shortcuts(self, &actions, ctx);
        }
    }

    /// The footprint's name, what its items are on, the symbol's pins, and the
    /// selected item's fields.
    pub fn properties(&mut self, ui: &mut egui::Ui) {
        // The whole Inspector scrolls, so nothing at its foot is lost below a
        // short pane; and it records where each of its widgets is for
        // [`Self::hits`]. The pane's own height is read before the scroll
        // area makes it endless: the two lists take their shares of it.
        let pane = ui.available_height();
        self.inspector.begin(ui.ctx());
        egui::ScrollArea::vertical()
            .id_salt("footprint_inspector")
            .auto_shrink([false, true])
            .show(ui, |ui| self.inspector_body(ui, pane));
    }
    fn inspector_body(&mut self, ui: &mut egui::Ui, pane: f32) {
        let before = self.footprint.clone();
        ui.label("Footprint name");
        let name = ui.text_edit_singleline(&mut self.footprint.name);
        self.inspector.mark("footprint_name", &name);
        ui.small(format!(
            "{} · {}",
            crate::count(self.footprint.pads.len(), "pad", "pads"),
            crate::count(self.footprint.silk.len(), "silkscreen line", "silkscreen lines")
        ));
        if !self.footprint.graphics.is_empty() {
            ui.small(crate::count(self.footprint.graphics.len(), "drawn item", "drawn items"));
        }
        // What the pads are ON, at a glance and before anything is selected,
        // because the failure this answers is that a user cannot tell.
        let (on_pins, mechanical, orphans) = self.link_counts();
        if self.pins_known() && !self.footprint.pads.is_empty() {
            ui.small(format!("{on_pins} on a pin · {mechanical} mechanical"));
        }
        if orphans > 0 {
            ui.label(
                egui::RichText::new(format!(
                    "{orphans} on no pin: {}",
                    self.orphan_numbers().join(", ")
                ))
                .color(ui.visuals().warn_fg_color),
            );
        }
        // Beside the tally, because it explains the same three states — and not
        // over the canvas, where a line that comes and goes with the first
        // mechanical pad would move the canvas under the pointer.
        self.legend(ui);
        // The grid a click snaps to, where the host shows it: the editor's own
        // toolbar, which carries it too, is not drawn in the app.
        ui.horizontal(|ui| {
            ui.label("Snap");
            self.grid_combo(ui, "footprint-grid-inspector");
        });
        // What the drawing tools draw on, and with what pen.
        if matches!(
            self.tool,
            DrawTool::Line | DrawTool::Rectangle | DrawTool::Circle | DrawTool::Arc | DrawTool::Text
        ) {
            let mut layer = self.draw_layer;
            ui.horizontal(|ui| {
                self.inspector.enum_dropdown(ui, "draw_layer", &layer_choices(), &mut layer);
                ui.label("Draw on");
            });
            if layer != self.draw_layer {
                self.draw_layer = layer;
                self.draw_width = None;
            }
            let mut width = self.draw_width.unwrap_or(layer.default_width());
            mm_field(ui, &mut self.inspector, "Pen width", &mut width, 0.001..=10.);
            self.draw_width = (width != layer.default_width()).then_some(width);
        }
        // The courtyard: the rectangle round the pads and silkscreen grown by
        // this clearance, which the board's design rule check keeps parts of
        // one side from overlapping. The default is stored as no value.
        let mut margin = self.footprint.courtyard_margin.unwrap_or(COURTYARD_MARGIN);
        mm_field(ui, &mut self.inspector, "Courtyard clearance", &mut margin, 0. ..=100.);
        self.footprint.courtyard_margin = (margin != COURTYARD_MARGIN).then_some(margin);
        // Plated holes whose copper ring is narrower than a board allows by
        // default, named here as the gap warning names close pads.
        let ring = DesignRules::default().min_annular_ring;
        let thin: Vec<String> = self
            .footprint
            .pads
            .iter()
            .enumerate()
            .filter(|(_, p)| p.annular_ring().is_some_and(|r| r < ring))
            .map(|(k, p)| if p.number.is_empty() { format!("mechanical pad {}", k + 1) } else { format!("pad {}", p.number) })
            .collect();
        if !thin.is_empty() {
            let warning = ui.label(
                egui::RichText::new(format!(
                    "Copper ring round the hole under {} mm: {}.",
                    mm_text(ring),
                    thin.join(", ")
                ))
                .color(ui.visuals().warn_fg_color),
            );
            self.inspector.mark("ring_warning", &warning);
        }
        // Pads closer than a board allows are found here, where they are drawn,
        // not on the board after the part is placed and routed.
        let close = self.close_pads();
        if let Some(&(i, j, gap)) = close.first() {
            let pads = &self.footprint.pads;
            let name = |k: usize| {
                let number = &pads[k].number;
                if number.is_empty() { format!("mechanical pad {}", k + 1) } else { format!("pad {number}") }
            };
            let more = match close.len() {
                1 => String::new(),
                n => format!(" ({} more pairs)", n - 1),
            };
            let warning = ui.label(
                egui::RichText::new(format!(
                    "{} and {} are {} mm apart; pads need {} mm between them{more}.",
                    name(i),
                    name(j),
                    mm_text(gap.round() as i32),
                    mm_text(min_pad_gap()),
                ))
                .color(ui.visuals().warn_fg_color),
            );
            self.inspector.mark("gap_warning", &warning);
        }
        // The selection came from elsewhere since the lists were last drawn.
        let reveal = self.core.selected != self.listed;
        self.pins_section(ui, pane, reveal);
        ui.separator();
        ui.label("Items");
        let notes: Vec<(Item, String, Option<Mark>)> = self
            .footprint
            .pads
            .iter()
            .enumerate()
            .map(|(i, pad)| {
                (
                    Item::Pad(i),
                    self.item_note(Item::Pad(i)),
                    link_ring(self.link(pad)).map(Mark::Ring),
                )
            })
            .chain((0..self.footprint.silk.len()).map(|i| {
                (Item::Silk(i), self.item_note(Item::Silk(i)), Some(Mark::Silk))
            }))
            .chain(self.footprint.graphics.iter().enumerate().map(|(i, g)| {
                (Item::Graphic(i), self.item_note(Item::Graphic(i)), Some(Mark::Drawn(graphic_color(g.layer))))
            }))
            .collect();
        // Read again: a pin's row just picked is a selection the Items list
        // has not shown.
        let reveal = self.core.selected != self.listed;
        let selected = self.core.selected;
        let mut pick = None;
        let keys: Vec<String> = self
            .footprint
            .pads
            .iter()
            .enumerate()
            .map(|(i, pad)| format!("row:{}", pad_key(i, pad)))
            .chain((0..self.footprint.silk.len()).map(|i| format!("row:silk:{i}")))
            .chain((0..self.footprint.graphics.len()).map(|i| format!("row:graphic:{i}")))
            .collect();
        let hits = &mut self.inspector;
        let list = egui::ScrollArea::vertical()
            .id_salt("footprint_items")
            .max_height(list_height(self.items_height, pane, LIST_SHARE, ITEMS_HEIGHT))
            .show(ui, |ui| {
                if notes.is_empty() {
                    ui.small("Nothing drawn yet. Place a pad with the SMD or Through-hole tool.");
                }
                let warn = ui.visuals().warn_fg_color;
                for ((item, note, mark), key) in notes.iter().zip(&keys) {
                    // The mark the item wears on the canvas, in the canvas's
                    // colours on its ground, so the row and the thing it names
                    // are recognisably one; the words in the theme's. A pad on
                    // no pin is in the theme's warning colour, as an unbound
                    // pin's row is in the Symbol Inspector, except on the
                    // selected row, whose own fill it does not read on.
                    let orphan = matches!(mark, Some(Mark::Ring(c)) if *c == WARNING);
                    let text = if orphan && selected != Some(*item) {
                        egui::RichText::new(note).color(warn)
                    } else {
                        egui::RichText::new(note)
                    };
                    let row = ui
                        .horizontal(|ui| {
                            swatch(ui, *mark);
                            ui.selectable_label(selected == Some(*item), text)
                        })
                        .inner;
                    if reveal && selected == Some(*item) {
                        row.scroll_to_me(Some(egui::Align::Center));
                    }
                    hits.mark(key, &row);
                    if row.clicked() {
                        pick = Some(*item);
                    }
                }
            });
        if let Some(grip) = grip_under(ui, &list, &mut self.items_height) {
            self.inspector.mark("items_grip", &grip);
        }
        if let Some(item) = pick {
            self.core.selected = Some(item);
        }
        // A row clicked is on screen already: only a selection made elsewhere
        // scrolls the lists.
        self.listed = self.core.selected;
        ui.separator();
        ui.label("Item properties");
        match self.core.selected {
            Some(Item::Pad(i)) if i < self.footprint.pads.len() => self.pad_properties(ui, i),
            Some(Item::Silk(i)) if i < self.footprint.silk.len() => self.silk_properties(ui, i),
            Some(Item::Graphic(i)) if i < self.footprint.graphics.len() => self.graphic_properties(ui, i),
            _ => {
                ui.small("Select an item on the canvas or in the list.");
            }
        }
        if self.core.selected.is_some() {
            let delete = ui.button("Delete item");
            self.inspector.mark("delete", &delete);
            if delete.clicked() {
                self.remove_selected();
            }
        }
        self.record_as(before, Some("pads:properties"));
    }
    /// The symbol's pins, each saying whether it has a pad — and offering the one
    /// click that gives it one. This is the list a user checks a footprint
    /// against: a pin with no pad is a part that cannot be routed, and the pads
    /// editor is where that is fixed.
    fn pins_section(&mut self, ui: &mut egui::Ui, pane: f32, reveal: bool) {
        if !self.pins_known() {
            return;
        }
        ui.separator();
        ui.label("Symbol pins");
        if self.pins.is_empty() {
            ui.small("This part's symbol has no pins, so every pad here is mechanical.");
            return;
        }
        let rows: Vec<(String, String, Option<usize>)> = self
            .pins
            .iter()
            .map(|pin| {
                let pad = self.footprint.pads.iter().position(|p| p.number == *pin);
                (pin.clone(), self.pin_row(pin), pad)
            })
            .collect();
        let selected = self.core.selected;
        let armed = self.pending_number.clone();
        let (mut pick, mut place, mut cancel) = (None, None, false);
        let hits = &mut self.inspector;
        let list = egui::ScrollArea::vertical()
            .id_salt("footprint_pins")
            .max_height(list_height(self.pins_height, pane, PINS_SHARE, PINS_HEIGHT))
            .show(ui, |ui| {
                for (pin, shown, pad) in &rows {
                    ui.horizontal_wrapped(|ui| match pad {
                        Some(index) => {
                            let row = ui
                                .selectable_label(
                                    selected == Some(Item::Pad(*index)),
                                    format!("{shown}  ·  pad"),
                                )
                                .on_hover_text("Select this pin's pad");
                            if reveal && selected == Some(Item::Pad(*index)) {
                                row.scroll_to_me(Some(egui::Align::Center));
                            }
                            hits.mark(format!("pin:{pin}:pad"), &row);
                            if row.clicked() {
                                pick = Some(*index);
                            }
                        }
                        None => {
                            ui.label(
                                egui::RichText::new(format!("{shown}  ·  no pad"))
                                    .color(ui.visuals().warn_fg_color),
                            );
                            // Named for its pin, so the next click on the canvas is
                            // unambiguous and so nothing has to be typed to match.
                            let waiting = armed.as_deref() == Some(pin.as_str());
                            let label = if waiting {
                                "Cancel".to_owned()
                            } else {
                                format!("Place pad for pin {pin}")
                            };
                            let button = ui.selectable_label(waiting, label).on_hover_text(format!(
                                "The next click on the canvas puts a pad there, already on pin {pin}"
                            ));
                            hits.mark(format!("place_pad:{pin}"), &button);
                            if button.clicked() {
                                if waiting {
                                    cancel = true;
                                } else {
                                    place = Some(pin.clone());
                                }
                            }
                        }
                    });
                }
            });
        if let Some(grip) = grip_under(ui, &list, &mut self.pins_height) {
            self.inspector.mark("pins_grip", &grip);
        }
        if let Some(index) = pick {
            self.core.selected = Some(Item::Pad(index));
        }
        if let Some(pin) = place {
            self.place_pad_for(pin);
        }
        if cancel {
            self.pending_number = None;
        }
        if self.missing_pins().is_empty() {
            ui.small("Every symbol pin has a pad.");
        }
    }
    /// A silkscreen line's fields, and the two true things about it: it is
    /// decoration, and it can be turned into a pad if that is what it should have
    /// been.
    fn silk_properties(&mut self, ui: &mut egui::Ui, i: usize) {
        ui.horizontal(|ui| {
            swatch(ui, Some(Mark::Silk));
            ui.label("Silkscreen line");
        });
        ui.small(
            "Decoration: it prints on the silkscreen film. It carries no pin and no net, so there is nothing to link it to.",
        );
        // Another layer or pen makes it a drawn item, which carries both.
        let mut layer = GraphicLayer::FrontSilk;
        ui.horizontal(|ui| {
            self.inspector.enum_dropdown(ui, "layer", &layer_choices(), &mut layer);
            ui.label("Layer");
        });
        let mut width = brep_ecad_core::fabrication::SILK_WIDTH;
        mm_field(ui, &mut self.inspector, "Line width", &mut width, 0.001..=10.);
        if layer != GraphicLayer::FrontSilk || width != brep_ecad_core::fabrication::SILK_WIDTH {
            let points = self.footprint.silk.remove(i);
            let mut graphic = LayerGraphic::new(layer, GraphicShape::Polyline { points });
            graphic.width = (width != layer.default_width()).then_some(width);
            self.core.selected = Some(Item::Graphic(self.footprint.graphics.len()));
            self.footprint.graphics.push(graphic);
            return;
        }
        let as_pad = silk_as_pad(&self.footprint.silk[i]).map(|(at, size, shape)| (at, size, shape, vec![])).or_else(|| {
            silk_as_custom_pad(&self.footprint.silk[i]).map(|(at, polygon)| {
                let (lo, hi) = polygon.iter().fold((polygon[0], polygon[0]), |(lo, hi), p| {
                    (Point::new(lo.x.min(p.x), lo.y.min(p.y)), Point::new(hi.x.max(p.x), hi.y.max(p.y)))
                });
                (at, Point::new(hi.x - lo.x, hi.y - lo.y), PadShape::Custom, polygon)
            })
        });
        if let Some((at, size, shape, polygon)) = as_pad {
            let number = self
                .missing_pins()
                .first()
                .map(|pin| (*pin).to_owned())
                .unwrap_or_else(|| self.next_number());
            let make = ui.button(format!("Make this a pad on pin {number}")).on_hover_text(
                "Replaces this outline with copper of the same size and place: a rectangle or circle pad, or a custom polygon pad for any other closed outline.",
            );
            self.inspector.mark("make_pad", &make);
            if make.clicked() {
                self.footprint.silk.remove(i);
                self.core.selected = Some(Item::Pad(self.footprint.pads.len()));
                self.footprint.pads.push(Pad {
                    number,
                    at,
                    size,
                    shape,
                    drill: None,
                    drill_height: None,
                    plated: true,
                    corner_radius: 0,
                    back: false,
                    layers: Default::default(),
                    trapezoid_delta: Default::default(),
                    polygon,
                    vias: None,
                });
                return;
            }
        }
        for (n, p) in self.footprint.silk[i].iter_mut().enumerate() {
            point_fields(ui, &mut self.inspector, &format!("Point {}", n + 1), p);
        }
    }
    /// A drawn item: its layer, pen, fill and, for text, its words and height.
    fn graphic_properties(&mut self, ui: &mut egui::Ui, i: usize) {
        edit_graphic(ui, &mut self.inspector, &mut self.footprint.graphics[i], &layer_choices());
    }
    fn pad_properties(&mut self, ui: &mut egui::Ui, i: usize) {
        // Read before the pad is held: the pin list, whether it can be judged at
        // all, and the pins with no pad of their own, which are what this pad may
        // be put on with one click.
        let pins = self.pins.clone();
        let known = self.pins_known();
        let free: Vec<String> = self.missing_pins().into_iter().map(str::to_owned).collect();
        let pad = &mut self.footprint.pads[i];
        let hits = &mut self.inspector;
        let mut kind = match (pad.drill, pad.plated) {
            (None, _) => PadKind::SurfaceMount,
            (Some(_), true) => PadKind::ThroughHole,
            (Some(_), false) => PadKind::Unplated,
        };
        // What the pad was before this frame's controls: the layer settings
        // below are tidied only when the user CHANGES its type, side or shape,
        // so selecting a pad never edits it.
        let (was_kind, was_back, was_shape) = (kind, pad.back, pad.shape);
        let choices = [
            (PadKind::SurfaceMount, "surface_mount", kind_name(PadKind::SurfaceMount)),
            (PadKind::ThroughHole, "through_hole", kind_name(PadKind::ThroughHole)),
            (PadKind::Unplated, "unplated", kind_name(PadKind::Unplated)),
        ];
        ui.horizontal(|ui| {
            hits.enum_dropdown(ui, "type", &choices, &mut kind);
            ui.label("Type");
        });
        let default_drill = (pad.size.x.min(pad.size.y) * 6 / 10 / 50 * 50).max(100);
        match kind {
            PadKind::SurfaceMount => {
                pad.drill = None;
                pad.drill_height = None;
                pad.plated = true;
            }
            PadKind::ThroughHole => {
                pad.drill = Some(pad.drill.unwrap_or(default_drill));
                pad.plated = true;
            }
            PadKind::Unplated => {
                if pad.plated || pad.drill.is_none() {
                    pad.number.clear();
                }
                pad.drill = Some(pad.drill.unwrap_or(pad.size.x.min(pad.size.y)));
                pad.plated = false;
            }
        }
        // The pad's PIN. A pad's link to a pin IS its number, so this is where the
        // link is made: stated first, then chosen by clicking a pin that has no
        // pad, then typed for a pin the symbol has not got yet, with Mechanical as
        // a CHOICE rather than an emptied field.
        ui.separator();
        if kind == PadKind::Unplated {
            ui.label("Pin");
            ui.small(link_note(PadLink::Mechanical));
            ui.small("An unplated hole is drilled, not plated: it cannot carry a pin.");
        } else {
            let link = link_of(&pad.number, &pins, known);
            ui.label("Pin");
            let colour = match link {
                PadLink::NoSuchPin => ui.visuals().warn_fg_color,
                _ => ui.visuals().weak_text_color(),
            };
            // A warning at body size, where it is the line most worth
            // reading; the plain note stays small and weak.
            let note = egui::RichText::new(link_note(link)).color(colour);
            ui.label(if link == PadLink::NoSuchPin { note } else { note.small() });
            ui.horizontal(|ui| {
                ui.label("Number");
                hits.mark("pad_number", &ui.text_edit_singleline(&mut pad.number));
            });
            if free.len() > PIN_BUTTONS {
                // On a forty-pin part a wall of buttons is not a choice; a list is.
                ui.small("Put this pad on a pin with no pad:");
                let choice = egui::ComboBox::from_id_salt("pad-pin-choice")
                    .selected_text("Choose a pin")
                    .show_ui(ui, |ui| {
                        for pin in &free {
                            if ui.selectable_label(false, pin).clicked() {
                                pad.number = pin.clone();
                            }
                        }
                    });
                hits.mark("choose_pin", &choice.response);
            } else if !free.is_empty() {
                ui.small("Pins with no pad — click one to put this pad on it:");
                ui.horizontal_wrapped(|ui| {
                    for pin in &free {
                        let button = ui.button(pin).on_hover_text(format!("Put this pad on pin {pin}"));
                        hits.mark(format!("put_on:{pin}"), &button);
                        if button.clicked() {
                            pad.number = pin.clone();
                        }
                    }
                });
            }
            if !pad.number.is_empty() {
                let button = ui.button("Make mechanical").on_hover_text(
                    "On no pin and no net, on purpose: a mounting hole, a shield tab, a castellation.",
                );
                hits.mark("make_mechanical", &button);
                if button.clicked() {
                    pad.number.clear();
                }
            }
        }
        ui.separator();
        let choices = [
            (PadShape::Rect, "rect", shape_name(PadShape::Rect)),
            (PadShape::RoundRect, "round_rect", shape_name(PadShape::RoundRect)),
            (PadShape::Circle, "circle", shape_name(PadShape::Circle)),
            (PadShape::Oval, "oval", shape_name(PadShape::Oval)),
            (PadShape::Trapezoid, "trapezoid", shape_name(PadShape::Trapezoid)),
            (PadShape::Custom, "custom", shape_name(PadShape::Custom)),
        ];
        ui.horizontal(|ui| {
            hits.enum_dropdown(ui, "shape", &choices, &mut pad.shape);
            ui.label("Shape");
        });
        if kind == PadKind::SurfaceMount {
            // Which copper of the footprint the pad is on. A part mounted on the
            // bottom flips the whole footprint, back pads included.
            let sides = [(false, "front", "Front (F.Cu)"), (true, "back", "Back (B.Cu)")];
            ui.horizontal(|ui| {
                hits.enum_dropdown(ui, "side", &sides, &mut pad.back);
                ui.label("Side");
            });
        } else if kind != was_kind {
            pad.back = false;
        }
        point_fields(ui, hits, "Position", &mut pad.at);
        if pad.shape != was_shape {
            // A pad made a rounded rectangle starts at a quarter of its short
            // side; one made anything else drops the radius it no longer uses.
            pad.corner_radius = if pad.shape == PadShape::RoundRect { pad.size.x.min(pad.size.y) / 4 } else { 0 };
            // A trapezoid starts with its bottom a quarter of the width longer
            // than its top; a custom pad starts as the box the pad filled, its
            // corners then moved one by one.
            pad.trapezoid_delta =
                if pad.shape == PadShape::Trapezoid { Point::new(0, pad.size.x / 4) } else { Point::default() };
            pad.polygon = if pad.shape == PadShape::Custom {
                let (x, y) = (pad.size.x / 2, pad.size.y / 2);
                vec![Point::new(-x, -y), Point::new(x, -y), Point::new(x, y), Point::new(-x, y)]
            } else {
                vec![]
            };
        }
        if pad.shape == PadShape::Custom {
            custom_pad_fields(ui, hits, pad);
        } else {
            ui.label("Size");
            mm_field(ui, hits, "Width", &mut pad.size.x, 0.01..=100.);
            mm_field(ui, hits, "Height", &mut pad.size.y, 0.01..=100.);
        }
        if pad.shape == PadShape::Trapezoid {
            ui.label("Taper").on_hover_text(
                "X lengthens the left side and shortens the right by as much; Y lengthens the bottom and shortens the top. Taper one pair of sides only.",
            );
            mm_field(ui, hits, "Taper X", &mut pad.trapezoid_delta.x, -100. ..=100.);
            mm_field(ui, hits, "Taper Y", &mut pad.trapezoid_delta.y, -100. ..=100.);
        }
        if pad.shape == PadShape::RoundRect {
            // Up to half the short side, or the stored radius if a resize left
            // it larger (it draws clamped): the field never clamps a loaded value.
            let most = (f64::from(pad.size.x.min(pad.size.y)) / 2.).max(f64::from(pad.corner_radius)) / 1000.;
            mm_field(ui, hits, "Corner radius", &mut pad.corner_radius, 0. ..=most.max(0.001));
        }
        if let Some(drill) = pad.drill {
            // Round, or a SLOT: the hole's width (x) and height (y) apart.
            let mut slot = pad.drill_height.is_some();
            let was_slot = slot;
            let holes = [(false, "round", "Round"), (true, "slot", "Slot (oval)")];
            ui.horizontal(|ui| {
                hits.enum_dropdown(ui, "hole", &holes, &mut slot);
                ui.label("Hole");
            });
            if slot != was_slot {
                if slot {
                    // Along the pad's long side, keeping the ring the round hole had.
                    let extra = (pad.size.x - pad.size.y).abs();
                    let long = if extra > 0 { drill + extra } else { (drill * 3 / 2).min(pad.size.y) };
                    if pad.size.x > pad.size.y {
                        (pad.drill, pad.drill_height) = (Some(long), Some(drill));
                    } else {
                        pad.drill_height = Some(long);
                    }
                } else {
                    pad.drill = Some(drill.min(pad.drill_height.unwrap_or(drill)));
                    pad.drill_height = None;
                }
            }
        }
        if let Some(height) = pad.drill_height.as_mut() {
            if let Some(drill) = pad.drill.as_mut() {
                mm_field(ui, hits, "Drill width", drill, 0.05..=100.);
            }
            mm_field(ui, hits, "Drill height", height, 0.05..=100.);
        } else if let Some(drill) = &mut pad.drill {
            mm_field(ui, hits, "Drill", drill, 0.05..=100.);
        }
        // What the preset puts on the mask and paste layers, and the overrides.
        ui.separator();
        ui.label("Solder mask and paste");
        let (front_cu, back_cu) = pad.copper_sides();
        let layers = &mut pad.layers;
        for (reaches, front, key, label) in [
            (front_cu, true, "mask_front", "Mask open on front"),
            (back_cu, false, "mask_back", "Mask open on back"),
        ] {
            if reaches {
                let cover = if front { &mut layers.cover_front } else { &mut layers.cover_back };
                let mut open = !*cover;
                let check = ui.checkbox(&mut open, label).on_hover_text(
                    "Off keeps the solder mask over the pad on this side (tented), e.g. against solder wicking into a via.",
                );
                hits.mark(key, &check);
                *cover = !open;
            }
        }
        let moved = kind != was_kind || pad_back_changed(was_back, front_cu, back_cu);
        if moved && !front_cu {
            layers.cover_front = false;
        }
        if moved && !back_cu {
            layers.cover_back = false;
        }
        let mut own_mask = layers.mask_margin.is_some();
        let check = ui.checkbox(&mut own_mask, "Own mask margin").on_hover_text(
            "Grow this pad's mask opening by its own margin instead of the board's solder mask expansion.",
        );
        hits.mark("own_mask_margin", &check);
        if own_mask {
            let mut m = layers.mask_margin.unwrap_or(DesignRules::default().mask_expansion);
            mm_field(ui, hits, "Mask margin", &mut m, -10. ..=10.);
            layers.mask_margin = Some(m);
        } else {
            layers.mask_margin = None;
        }
        if kind == PadKind::SurfaceMount {
            let mut paste = !layers.no_paste;
            let check = ui.checkbox(&mut paste, "Solder paste").on_hover_text(
                "Off leaves this pad out of the paste stencil, e.g. a large thermal pad pasted through smaller paste-only pads laid over it.",
            );
            hits.mark("paste", &check);
            layers.no_paste = !paste;
            if paste {
                let mut own = layers.paste_margin.is_some();
                let check = ui.checkbox(&mut own, "Own paste margin").on_hover_text(
                    "Grow (or, negative, shrink) the paste aperture from the copper outline.",
                );
                hits.mark("own_paste_margin", &check);
                if own {
                    let mut m = layers.paste_margin.unwrap_or(0);
                    mm_field(ui, hits, "Paste margin", &mut m, -10. ..=10.);
                    layers.paste_margin = Some(m);
                } else {
                    layers.paste_margin = None;
                }
                let mut windowed = layers.paste_windows.is_some();
                let check = ui.checkbox(&mut windowed, "Windowpane paste").on_hover_text(
                    "Split the paste into a grid of smaller windows, as a large thermal pad's is, so the part does not float on one deposit. A window that would leave the pad is not cut.",
                );
                hits.mark("paste_windows", &check);
                if windowed {
                    let mut windows = layers.paste_windows.unwrap_or(PasteWindows { columns: 2, rows: 2, gap: 200 });
                    for (key, label, n) in [("paste_columns", "Columns", &mut windows.columns), ("paste_rows", "Rows", &mut windows.rows)] {
                        ui.horizontal(|ui| {
                            ui.label(label);
                            let field = ui.add(egui::DragValue::new(n).range(1..=20));
                            hits.mark(key, &field);
                        });
                    }
                    mm_field(ui, hits, "Window gap", &mut windows.gap, 0. ..=10.);
                    layers.paste_windows = Some(windows);
                } else {
                    layers.paste_windows = None;
                }
            } else {
                layers.paste_windows = None;
            }
        } else if kind != was_kind {
            layers.no_paste = false;
            layers.paste_margin = None;
            layers.paste_windows = None;
        }
        if kind == PadKind::SurfaceMount {
            // A thermal pad's vias, through the board under it.
            let mut gridded = pad.vias.is_some();
            let check = ui.checkbox(&mut gridded, "Vias in the pad").on_hover_text(
                "A grid of through vias in this pad, carried by the footprint: heat and a ground connection to the other layers, as a thermal pad's.",
            );
            hits.mark("pad_vias", &check);
            if gridded {
                let mut grid = pad.vias.unwrap_or_else(|| {
                    let fit = |side: i32| u8::try_from(((side - 600) / 1000 + 1).clamp(1, 20)).unwrap_or(1);
                    PadVias { columns: fit(pad.size.x), rows: fit(pad.size.y), pitch: Point::new(1000, 1000), drill: 300, diameter: 600, tented: true }
                });
                for (key, label, n) in [("via_columns", "Via columns", &mut grid.columns), ("via_rows", "Via rows", &mut grid.rows)] {
                    ui.horizontal(|ui| {
                        ui.label(label);
                        let field = ui.add(egui::DragValue::new(n).range(1..=20));
                        hits.mark(key, &field);
                    });
                }
                mm_field(ui, hits, "Via pitch X", &mut grid.pitch.x, 0.05..=100.);
                mm_field(ui, hits, "Via pitch Y", &mut grid.pitch.y, 0.05..=100.);
                mm_field(ui, hits, "Via drill", &mut grid.drill, 0.05..=10.);
                mm_field(ui, hits, "Via diameter", &mut grid.diameter, 0.05..=20.);
                let mut tented = grid.tented;
                let check = ui.checkbox(&mut tented, "Tent vias on the far side").on_hover_text(
                    "Keeps the solder mask over the vias on the side away from the pad, so solder cannot wick through. On the pad's own side its opening shows them.",
                );
                hits.mark("tent_pad_vias", &check);
                grid.tented = tented;
                pad.vias = Some(grid);
                if let Some(problem) = pad.vias_problem() {
                    ui.colored_label(ui.visuals().warn_fg_color, format!("This pad {problem}."));
                }
            } else {
                pad.vias = None;
            }
        } else {
            pad.vias = None;
        }
        let rotate = ui.button("Rotate pad 90°");
        hits.mark("rotate", &rotate);
        if rotate.clicked() {
            pad.size = Point::new(pad.size.y, pad.size.x);
            // A quarter turn, (x, y) to (-y, x): a trapezoid's taper and a
            // custom pad's corners turn with it.
            pad.trapezoid_delta = Point::new(pad.trapezoid_delta.y, -pad.trapezoid_delta.x);
            for p in &mut pad.polygon {
                *p = Point::new(-p.y, p.x);
            }
            // A windowpane's columns run where its rows did, and a via grid's.
            if let Some(windows) = pad.layers.paste_windows.as_mut() {
                (windows.columns, windows.rows) = (windows.rows, windows.columns);
            }
            if let Some(grid) = pad.vias.as_mut() {
                (grid.columns, grid.rows) = (grid.rows, grid.columns);
                grid.pitch = Point::new(grid.pitch.y, grid.pitch.x);
            }
            // A slot turns with its pad.
            if let (Some(width), Some(height)) = (pad.drill, pad.drill_height) {
                (pad.drill, pad.drill_height) = (Some(height), Some(width));
            }
        }
        ui.separator();
        ui.label("Repeat pad");
        ui.horizontal(|ui| {
            ui.label("Copies");
            let copies = ui.add(egui::DragValue::new(&mut self.repeat.0).range(1..=200));
            hits.mark("copies", &copies);
        });
        mm_field(ui, hits, "Step X", &mut self.repeat.1, -100. ..=100.);
        mm_field(ui, hits, "Step Y", &mut self.repeat.2, -100. ..=100.);
        let add = ui
            .add_enabled(
                self.repeat.1 != 0 || self.repeat.2 != 0,
                egui::Button::new("Add copies"),
            )
            .on_hover_text("Copies continue the pad numbering, e.g. 1 -> 2, 3, 4.");
        hits.mark("add_copies", &add);
        if add.clicked() {
            self.repeat_pad(i);
        }
    }
    fn repeat_pad(&mut self, i: usize) {
        let (count, dx, dy) = self.repeat;
        let source = self.footprint.pads[i].clone();
        let mut number = source.number.parse::<u32>().ok();
        for k in 1..=count as i32 {
            let mut pad = source.clone();
            pad.at = Point::new(source.at.x + dx * k, source.at.y + dy * k);
            if !source.number.is_empty() {
                pad.number = match number {
                    Some(n) => {
                        let next = (n + 1..)
                            .find(|m| {
                                !self
                                    .footprint
                                    .pads
                                    .iter()
                                    .any(|p| p.number == m.to_string())
                            })
                            .unwrap_or(n + 1);
                        number = Some(next);
                        next.to_string()
                    }
                    None => self.next_number(),
                };
            }
            self.footprint.pads.push(pad);
        }
        self.core.selected = Some(Item::Pad(self.footprint.pads.len() - 1));
    }
    fn remove_selected(&mut self) {
        match self.core.selected.take() {
            Some(Item::Pad(i)) if i < self.footprint.pads.len() => {
                self.footprint.pads.remove(i);
            }
            Some(Item::Silk(i)) if i < self.footprint.silk.len() => {
                self.footprint.silk.remove(i);
            }
            Some(Item::Graphic(i)) if i < self.footprint.graphics.len() => {
                self.footprint.graphics.remove(i);
            }
            _ => {}
        }
    }
    fn hit(&self, p: Pos2, screen: impl Fn(Point) -> Pos2, world: Point) -> Option<Item> {
        let tolerance = f64::from(5. / self.core.zoom);
        for (i, pad) in self.footprint.pads.iter().enumerate().rev() {
            if pad.local_shape().distance_to_point(world) <= tolerance {
                return Some(Item::Pad(i));
            }
        }
        for (i, line) in self.footprint.silk.iter().enumerate().rev() {
            if line
                .windows(2)
                .any(|s| segment_distance(p, screen(s[0]), screen(s[1])) < 8.)
            {
                return Some(Item::Silk(i));
            }
        }
        for (i, g) in self.footprint.graphics.iter().enumerate().rev() {
            let strokes = g.strokes(!g.layer.front());
            let near = strokes.iter().any(|line| {
                line.len() == 1 && p.distance(screen(line[0])) < 8.
                    || line.windows(2).any(|s| segment_distance(p, screen(s[0]), screen(s[1])) < 8.)
            });
            // Text answers anywhere in the box its strokes fill.
            let boxed = matches!(g.shape, GraphicShape::Text { .. }) && {
                let points: Vec<Point> = strokes.iter().flatten().copied().collect();
                let (lo, hi) = (
                    points.iter().fold(Point::new(i32::MAX, i32::MAX), |m, p| Point::new(m.x.min(p.x), m.y.min(p.y))),
                    points.iter().fold(Point::new(i32::MIN, i32::MIN), |m, p| Point::new(m.x.max(p.x), m.y.max(p.y))),
                );
                Rect::from_two_pos(screen(lo), screen(hi)).expand(4.).contains(p)
            };
            let inside = boxed || g.filled && strokes.first().is_some_and(|line| point_in(world, line));
            if near || inside {
                return Some(Item::Graphic(i));
            }
        }
        None
    }
    fn canvas(&mut self, ui: &mut egui::Ui, size: Vec2) {
        self.canvas_pass = ui.ctx().cumulative_pass_nr();
        let (response, painter) = ui.allocate_painter(size, Sense::click_and_drag());
        let rect = response.rect;
        self.canvas = rect;
        if self.fit {
            let (mut min, mut max) = self.footprint.bounds();
            min = Point::new(min.x.min(-2000), min.y.min(-2000));
            max = Point::new(max.x.max(2000), max.y.max(2000));
            self.core.zoom = crate::valid_zoom(((rect.width() - 70.) / (max.x - min.x) as f32)
                .min((rect.height() - 70.) / (max.y - min.y) as f32)
                .min(1.), self.core.zoom);
            self.core.pan =
                -Vec2::new((min.x + max.x) as f32 / 2., (min.y + max.y) as f32 / 2.) * self.core.zoom;
            self.fit = false;
        }
        if response.dragged_by(PointerButton::Secondary)
            || response.dragged_by(PointerButton::Middle)
        {
            self.core.pan += ui.input(|i| i.pointer.delta());
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            self.core.zoom = crate::scaled_zoom(self.core.zoom, (scroll * 0.002).exp());
        }
        let zoom = self.core.zoom;
        let center = rect.center() + self.core.pan;
        let screen = |p: Point| on_canvas(center, zoom, p);
        let raw = |p: Pos2| {
            Point::new(
                ((p.x - center.x) / zoom).round() as i32,
                ((p.y - center.y) / zoom).round() as i32,
            )
        };
        let pointer = response.interact_pointer_pos();
        if response.drag_started_by(PointerButton::Primary)
            && let Some(p) = ui.input(|i| i.pointer.press_origin())
        {
            if self.tool == DrawTool::Select {
                self.core.selected = self.hit(p, screen, raw(p));
                self.core.drag = self.core.selected.map(|_| (self.footprint.clone(), raw(p)));
            } else {
                self.start = Some(self.snap(raw(p)));
            }
        }
        if let (Some((before, start)), Some(p)) = (&self.core.drag, pointer) {
            let q = raw(p);
            // Snap the offset so items that sit off the grid keep their position relative to it.
            let delta = self.snap(Point::new(q.x - start.x, q.y - start.y));
            self.footprint = before.clone();
            match self.core.selected {
                Some(Item::Pad(i)) => {
                    let pad = &mut self.footprint.pads[i];
                    pad.at = pad.at.offset(delta);
                }
                Some(Item::Silk(i)) => {
                    for p in &mut self.footprint.silk[i] {
                        *p = p.offset(delta);
                    }
                }
                Some(Item::Graphic(i)) => move_graphic(&mut self.footprint.graphics[i], delta),
                None => {}
            }
        }
        if response.drag_stopped_by(PointerButton::Primary) {
            if let Some((before, _)) = self.core.drag.take() {
                self.record(before);
            }
            if let Some(start) = self.start.take()
                && let Some(p) = pointer
            {
                let end = self.snap(raw(p));
                self.add_item(start, end);
            }
        }
        if response.clicked()
            && let Some(p) = pointer
        {
            match self.tool {
                DrawTool::Pad | DrawTool::ThroughHole => {
                    let at = self.snap(raw(p));
                    self.add_item(at, at);
                }
                DrawTool::Text => {
                    let at = self.snap(raw(p));
                    self.add_item(at, at);
                }
                DrawTool::Select => self.core.selected = self.hit(p, screen, raw(p)),
                _ => {}
            }
        }
        if response.secondary_clicked()
            && let Some(p) = pointer
        {
            self.core.selected = self.hit(p, screen, raw(p));
        }
        response.context_menu(|ui| {
            menu_ui(ui);
            if self.core.selected.is_some() && ui.button("Delete item").clicked() {
                let before = self.footprint.clone();
                self.remove_selected();
                self.record(before);
                ui.close();
            }
        });

        painter.rect_filled(rect, 0., BACKGROUND);
        let painter = painter.with_clip_rect(rect);
        let step = self.grid as f32 * zoom;
        if step > 6. {
            let mut x = rect.left() + (center.x - rect.left()).rem_euclid(step);
            while x < rect.right() {
                let mut y = rect.top() + (center.y - rect.top()).rem_euclid(step);
                while y < rect.bottom() {
                    painter.circle_filled(Pos2::new(x, y), 0.8, Color32::from_gray(52));
                    y += step;
                }
                x += step;
            }
        }
        for (a, b) in [
            (Vec2::new(10., 0.), Vec2::new(-10., 0.)),
            (Vec2::new(0., 10.), Vec2::new(0., -10.)),
        ] {
            painter.line_segment([center + a, center + b], Stroke::new(1., Color32::GRAY));
        }
        if self.footprint.drawn_courtyard(true).is_none()
            && (!self.footprint.pads.is_empty() || !self.footprint.silk.is_empty())
        {
            let (min, max) = self.footprint.bounds();
            let margin = self.footprint.courtyard_margin.unwrap_or(COURTYARD_MARGIN);
            let margin = Point::new(margin, margin);
            let courtyard = Rect::from_two_pos(
                screen(Point::new(min.x - margin.x, min.y - margin.y)),
                screen(max.offset(margin)),
            );
            painter.rect_stroke(
                courtyard,
                0.,
                Stroke::new(1., Color32::from_gray(90)),
                egui::StrokeKind::Middle,
            );
        }
        // `outline` is drawn outside the shape, `grow` points clear of its edge.
        let paint_shape = |shape: Shape, fill: Option<Color32>, outline: Option<Stroke>, grow: f32| match shape
        {
            // egui keeps a rectangle's corner radius in a byte, so a corner wider
            // than that on screen is drawn as the pad's own outline instead.
            Shape::RoundRect { radius, .. } if radius as f32 * zoom > MAX_PAINTED_CORNER => {
                if let Some(fill) = fill {
                    let ring: Vec<Pos2> = shape.outline(16).into_iter().map(screen).collect();
                    painter.add(egui::Shape::convex_polygon(ring, fill, Stroke::NONE));
                }
                if let Some(stroke) = outline {
                    let reach = ((grow + stroke.width / 2.) / zoom).round() as i32;
                    let ring: Vec<Pos2> = shape.grown(reach).outline(16).into_iter().map(screen).collect();
                    painter.add(egui::Shape::closed_line(ring, stroke));
                }
            }
            // A trapezoid or custom pad, which may turn inward: filled by ear
            // clipping, its outline the polygon grown past the edge.
            Shape::Polygon { .. } => {
                if let Some(fill) = fill {
                    let ring: Vec<Pos2> = shape.outline(16).into_iter().map(screen).collect();
                    crate::board_view::paint_filled(&painter, &ring, fill);
                }
                if let Some(stroke) = outline {
                    let reach = ((grow + stroke.width / 2.) / zoom).round() as i32;
                    let ring: Vec<Pos2> = shape.grown(reach).outline(16).into_iter().map(screen).collect();
                    painter.add(egui::Shape::closed_line(ring, stroke));
                }
            }
            Shape::Rect { min, max } | Shape::RoundRect { min, max, .. } => {
                let r = Rect::from_two_pos(screen(min), screen(max));
                let corner = match shape {
                    Shape::RoundRect { radius, .. } => radius as f32 * zoom,
                    _ => 0.,
                };
                if let Some(fill) = fill {
                    painter.rect_filled(r, corner, fill);
                }
                if let Some(stroke) = outline {
                    let corner = if corner > 0. { corner + grow } else { 0. };
                    painter.rect_stroke(r.expand(grow), corner, stroke, egui::StrokeKind::Outside);
                }
            }
            Shape::Capsule { a, b, radius } => {
                let r = (radius as f32 * zoom).max(0.5);
                let (a, b) = (screen(a), screen(b));
                if let Some(fill) = fill {
                    if a != b {
                        painter.line_segment([a, b], Stroke::new(2. * r, fill));
                        painter.circle_filled(b, r, fill);
                    }
                    painter.circle_filled(a, r, fill);
                }
                if let Some(stroke) = outline {
                    let r = r + grow + stroke.width / 2.;
                    if a == b {
                        painter.circle_stroke(a, r, stroke);
                    } else {
                        let n = (b - a).normalized().rot90() * r;
                        painter.line_segment([a + n, b + n], stroke);
                        painter.line_segment([a - n, b - n], stroke);
                        painter.circle_stroke(a, r, stroke);
                        painter.circle_stroke(b, r, stroke);
                    }
                }
            }
        };
        for (i, pad) in self.footprint.pads.iter().enumerate() {
            let shape = pad.local_shape();
            let fill = match (pad.drill, pad.plated) {
                (None, _) => Some(layer_color(u8::from(pad.back), 2)),
                (Some(_), true) => Some(THROUGH_HOLE),
                (Some(_), false) => None,
            };
            paint_shape(shape.clone(), fill, None, 0.);
            if let Some(grid) = pad.vias {
                for at in pad.via_points() {
                    painter.circle_filled(screen(at), (grid.diameter as f32 * zoom / 2.).max(1.), THROUGH_HOLE);
                    painter.circle_filled(screen(at), (grid.drill as f32 * zoom / 2.).max(0.5), BACKGROUND);
                }
            }
            if let Some(hole) = pad.local_hole() {
                let edge = (!pad.plated).then(|| Stroke::new(1., SILK));
                paint_shape(hole, Some(BACKGROUND), edge, 0.);
            }
            // What the pad is ON, as a ring: grey for a mechanical pad, the
            // warning colour for a number no pin of the symbol carries. A pad on
            // its pin wears nothing, because that is the ordinary case and a mark
            // on every pad marks nothing. The ring stands CLEAR of the pad, on
            // the canvas's dark ground: laid on the pad's red edge, as it was,
            // the orange measured 1.61:1 against the red and did not read at the
            // runner's zoom, and a selection's outline covered it outright. Here
            // it measures 6.65:1 against the ground, and the selection's outline
            // fills the gap inside it.
            if let Some(colour) = link_ring(self.link(pad)) {
                paint_shape(shape.clone(), None, Some(Stroke::new(RING_WIDTH, colour)), RING_GAP);
            }
            if self.core.selected == Some(Item::Pad(i)) {
                paint_shape(shape, None, Some(Stroke::new(2., ACCENT)), 0.);
            }
        }
        // Pads closer than a board allows: a line joining the pair, in the
        // warning colour, so the pair the Inspector names is the pair on the
        // canvas. Under the numbers, which stay legible on it.
        for (i, j, _) in self.close_pads() {
            let (a, b) = (&self.footprint.pads[i], &self.footprint.pads[j]);
            painter.line_segment([screen(a.at), screen(b.at)], Stroke::new(3., WARNING));
        }
        // Numbers go on top so neighbouring holes and pads never hide them.
        for pad in self.footprint.pads.iter().filter(|p| !p.number.is_empty()) {
            let height = (pad.size.x.min(pad.size.y) as f32 * zoom * 0.5).clamp(8., 16.);
            painter.text(
                screen(pad.at),
                Align2::CENTER_CENTER,
                &pad.number,
                FontId::monospace(height),
                // White on every pad: the warning orange on the pad's red
                // measured 1.61:1, white 4.50:1. The ring says the rest.
                Color32::WHITE,
            );
        }
        for (i, line) in self.footprint.silk.iter().enumerate() {
            let color = if self.core.selected == Some(Item::Silk(i)) {
                ACCENT
            } else {
                SILK
            };
            painter.add(egui::Shape::line(
                line.iter().map(|p| screen(*p)).collect(),
                Stroke::new(1.5, color),
            ));
        }
        // Drawn items with their own pens, the selected one in the accent.
        for (i, g) in self.footprint.graphics.iter().enumerate() {
            let color = if self.core.selected == Some(Item::Graphic(i)) { ACCENT } else { graphic_color(g.layer) };
            let width = (g.width() as f32 * zoom).max(1.);
            let strokes = g.strokes(!g.layer.front());
            if g.filled
                && let Some(line) = strokes.first()
            {
                let outline: Vec<Pos2> = line.iter().map(|p| screen(*p)).collect();
                crate::board_view::paint_filled(&painter, &outline, color.gamma_multiply(0.6));
            }
            for line in strokes {
                painter.add(egui::Shape::line(line.iter().map(|p| screen(*p)).collect(), Stroke::new(width, color)));
            }
        }
        // Hovering names the item under the pointer and says what it is on. The
        // canvas is where a user looks first, and a number drawn on a pad does not
        // say whether the symbol has a pin of that number.
        if self.core.drag.is_none()
            && self.start.is_none()
            && let Some(p) = ui
                .input(|i| i.pointer.hover_pos())
                .filter(|p| rect.contains(*p))
            && let Some(item) = self.hit(p, screen, raw(p))
        {
            let note = match item {
                Item::Pad(i) => match self.footprint.pads.get(i) {
                    Some(pad) => format!("{}\n{}", self.item_note(item), link_note(self.link(pad))),
                    None => self.item_note(item),
                },
                Item::Silk(_) => format!("{}\nNo pin and no net: it prints on the silkscreen film.", self.item_note(item)),
                Item::Graphic(_) => format!("{}\nNo pin and no net: drawing on a non-copper layer.", self.item_note(item)),
            };
            response.clone().on_hover_text_at_pointer(note);
        }
        // Where a click would put the thing being drawn: the SNAPPED point, which
        // is not where the pointer is. At 50 µm a pointer at 1.27 mm put its pad
        // at 1.25, and nothing said so until the Inspector's Position field.
        if self.tool != DrawTool::Select
            && self.core.drag.is_none()
            && let Some(p) = ui.input(|i| i.pointer.hover_pos()).filter(|p| rect.contains(*p))
        {
            let at = self.snap(raw(p));
            let c = screen(at);
            let stroke = Stroke::new(1., ACCENT);
            painter.line_segment([c - Vec2::new(6., 0.), c + Vec2::new(6., 0.)], stroke);
            painter.line_segment([c - Vec2::new(0., 6.), c + Vec2::new(0., 6.)], stroke);
            painter.text(
                c + Vec2::new(8., -8.),
                Align2::LEFT_BOTTOM,
                format!("{}, {} mm", mm_text(at.x), mm_text(at.y)),
                FontId::proportional(12.),
                ACCENT,
            );
        }
        if let (Some(start), Some(p)) = (self.start, pointer) {
            let a = screen(start);
            let b = screen(self.snap(raw(p)));
            let stroke = Stroke::new(1.5, ACCENT);
            match self.tool {
                DrawTool::Circle => {
                    painter.circle_stroke(a, a.distance(b), stroke);
                }
                DrawTool::Line => {
                    painter.line_segment([a, b], stroke);
                }
                DrawTool::Arc => {
                    let end = self.snap(raw(p));
                    let points = brep_ecad_core::graphic::arc_points(start, arc_through(start, end), end);
                    painter.add(egui::Shape::line(points.into_iter().map(&screen).collect(), stroke));
                }
                _ => {
                    painter.rect_stroke(
                        Rect::from_two_pos(a, b),
                        0.,
                        stroke,
                        egui::StrokeKind::Middle,
                    );
                }
            }
        }
    }
    fn add_item(&mut self, a: Point, b: Point) {
        let pad_tool = matches!(self.tool, DrawTool::Pad | DrawTool::ThroughHole);
        if a == b && !pad_tool && self.tool != DrawTool::Text {
            return;
        }
        let before = self.footprint.clone();
        match self.tool {
            DrawTool::Pad | DrawTool::ThroughHole => {
                let through = self.tool == DrawTool::ThroughHole;
                let size = if a == b {
                    if through { THT_SIZE } else { SMD_SIZE }
                } else {
                    Point::new((b.x - a.x).abs().max(50), (b.y - a.y).abs().max(50))
                };
                let at = Point::new(
                    ((i64::from(a.x) + i64::from(b.x)) / 2) as i32,
                    ((i64::from(a.y) + i64::from(b.y)) / 2) as i32,
                );
                // A pad placed FOR a pin carries that pin's number from the frame
                // it comes into existence; any other pad takes the lowest free
                // number, as it always did.
                let number = match self.pending_number.take() {
                    Some(pin) if !self.footprint.pads.iter().any(|p| p.number == pin) => pin,
                    _ => self.next_number(),
                };
                self.core.selected = Some(Item::Pad(self.footprint.pads.len()));
                self.footprint.pads.push(Pad {
                    number,
                    at,
                    size,
                    shape: match (through, size.x == size.y) {
                        (false, _) => PadShape::Rect,
                        (true, true) => PadShape::Circle,
                        (true, false) => PadShape::Oval,
                    },
                    drill: through.then(|| (size.x.min(size.y) * 6 / 10 / 50 * 50).max(100)),
                    drill_height: None,
                    plated: true,
                    corner_radius: 0,
                    back: false,
                    layers: Default::default(),
                    trapezoid_delta: Default::default(),
                    polygon: vec![],
                    vias: None,
                });
            }
            DrawTool::Line | DrawTool::Rectangle | DrawTool::Circle | DrawTool::Arc | DrawTool::Text
                if self.draw_layer != GraphicLayer::FrontSilk
                    || self.draw_width.is_some()
                    || matches!(self.tool, DrawTool::Arc | DrawTool::Text) =>
            {
                // On another layer or pen, or a shape only a drawn item keeps
                // exactly: a drawn item.
                let shape = match self.tool {
                    DrawTool::Line => GraphicShape::Polyline { points: vec![a, b] },
                    DrawTool::Rectangle => GraphicShape::Polyline {
                        points: vec![a, Point::new(b.x, a.y), b, Point::new(a.x, b.y), a],
                    },
                    DrawTool::Circle => GraphicShape::Circle {
                        center: a,
                        radius: f64::from(b.x - a.x).hypot(f64::from(b.y - a.y)).round().min(1e8) as i32,
                    },
                    DrawTool::Arc => GraphicShape::Arc { start: a, mid: arc_through(a, b), end: b },
                    _ => GraphicShape::Text { at: a, text: "TEXT".into(), height: 1000 },
                };
                let mut graphic = LayerGraphic::new(self.draw_layer, shape);
                graphic.width = self.draw_width;
                if graphic.problem().is_some() {
                    return;
                }
                self.core.selected = Some(Item::Graphic(self.footprint.graphics.len()));
                self.footprint.graphics.push(graphic);
            }
            DrawTool::Line | DrawTool::Rectangle | DrawTool::Circle => {
                let line = match self.tool {
                    DrawTool::Line => vec![a, b],
                    DrawTool::Rectangle => {
                        vec![a, Point::new(b.x, a.y), b, Point::new(a.x, b.y), a]
                    }
                    _ => {
                        let r = f64::from(b.x - a.x).hypot(f64::from(b.y - a.y));
                        (0..=36)
                            .map(|i| {
                                let t = std::f64::consts::TAU * f64::from(i) / 36.;
                                Point::new(
                                    a.x + (r * t.cos()).round() as i32,
                                    a.y + (r * t.sin()).round() as i32,
                                )
                            })
                            .collect()
                    }
                };
                self.core.selected = Some(Item::Silk(self.footprint.silk.len()));
                self.footprint.silk.push(line);
            }
            DrawTool::Select | DrawTool::Arc | DrawTool::Text => {}
        }
        self.record(before);
    }
}
/// A drawn item's Inspector: what it is, its layer (from `layers`), its pen,
/// its fill when it is closed, and its points or text, edited in place. Shared
/// by the Pads editor and the board's own drawing.
pub(crate) fn edit_graphic(
    ui: &mut egui::Ui,
    hits: &mut InspectorHits,
    graphic: &mut LayerGraphic,
    layers: &[(GraphicLayer, &'static str, &'static str)],
) {
    ui.horizontal(|ui| {
        swatch(ui, Some(Mark::Drawn(graphic_color(graphic.layer))));
        ui.label(graphic_kind(graphic));
    });
    ui.small(match graphic.layer {
        GraphicLayer::FrontSilk | GraphicLayer::BackSilk => "Prints on the silkscreen film of its side. No pin and no net.",
        GraphicLayer::FrontFab | GraphicLayer::BackFab => "Fabrication drawing: shown in the editors, never manufactured.",
        GraphicLayer::FrontCourtyard => "Courtyard: the room the part needs on the side it is mounted on. It replaces the generated rectangle, and the design rule check keeps parts' courtyards apart.",
        _ => "Back courtyard: room the part needs on the board's other side, which the design rule check keeps apart from the parts mounted there.",
    });
    let mut layer = graphic.layer;
    ui.horizontal(|ui| {
        hits.enum_dropdown(ui, "layer", layers, &mut layer);
        ui.label("Layer");
    });
    if layer != graphic.layer {
        // A pen at the old layer's own width takes the new layer's.
        graphic.layer = layer;
    }
    let mut width = graphic.width();
    mm_field(ui, hits, "Line width", &mut width, 0.001..=10.);
    graphic.width = (width != layer.default_width()).then_some(width);
    if graphic.closed() {
        let fill = ui.checkbox(&mut graphic.filled, "Filled");
        hits.mark("filled", &fill);
    }
    match &mut graphic.shape {
        GraphicShape::Text { at, text, height } => {
            ui.label("Text");
            let field = ui.text_edit_singleline(text);
            hits.mark("text", &field);
            mm_field(ui, hits, "Text height", height, 0.1..=100.);
            point_fields(ui, hits, "Position", at);
        }
        GraphicShape::Circle { center, radius } => {
            point_fields(ui, hits, "Centre", center);
            mm_field(ui, hits, "Radius", radius, 0.001..=100_000.);
        }
        GraphicShape::Arc { start, mid, end } => {
            point_fields(ui, hits, "Start", start);
            point_fields(ui, hits, "Through", mid);
            point_fields(ui, hits, "End", end);
        }
        GraphicShape::Polyline { points } => {
            // A closed shape's last point IS its first: it is offered once, and
            // the two move together, so an edit never opens the shape.
            let closed = points.len() > 2 && points.first() == points.last();
            let shown = if closed { points.len() - 1 } else { points.len() };
            for (n, p) in points[..shown].iter_mut().enumerate() {
                point_fields(ui, hits, &format!("Point {}", n + 1), p);
            }
            if closed {
                let first = points[0];
                *points.last_mut().expect("a closed shape has points") = first;
            }
        }
    }
}
/// The layers a drawn item can be on, as the Inspector's dropdowns offer them.
pub(crate) fn layer_choices() -> Vec<(GraphicLayer, &'static str, &'static str)> {
    GraphicLayer::ALL
        .iter()
        .map(|&layer| {
            let label = match layer {
                GraphicLayer::FrontSilk => "Front silkscreen (F.SilkS)",
                GraphicLayer::BackSilk => "Back silkscreen (B.SilkS)",
                GraphicLayer::FrontFab => "Front fabrication (F.Fab)",
                GraphicLayer::BackFab => "Back fabrication (B.Fab)",
                GraphicLayer::FrontCourtyard => "Front courtyard (F.CrtYd)",
                GraphicLayer::BackCourtyard => "Back courtyard (B.CrtYd)",
            };
            (layer, layer.name(), label)
        })
        .collect()
}
/// What a drawn item is, in a word.
pub(crate) fn graphic_kind(graphic: &LayerGraphic) -> &'static str {
    match &graphic.shape {
        GraphicShape::Polyline { points } if points.len() == 5 && points.first() == points.last() => "Rectangle",
        GraphicShape::Polyline { .. } if graphic.closed() => "Polygon",
        GraphicShape::Polyline { .. } => "Line",
        GraphicShape::Arc { .. } => "Arc",
        GraphicShape::Circle { .. } => "Circle",
        GraphicShape::Text { .. } => "Text",
    }
}
/// The point a half-circle arc from `a` to `b` passes through: the Arc tool's
/// drag is the chord, and the arc bulges to its left.
fn arc_through(a: Point, b: Point) -> Point {
    let (mx, my) = ((i64::from(a.x) + i64::from(b.x)) / 2, (i64::from(a.y) + i64::from(b.y)) / 2);
    let (dx, dy) = ((i64::from(b.x) - i64::from(a.x)) / 2, (i64::from(b.y) - i64::from(a.y)) / 2);
    let clamp = |v: i64| v.clamp(-100_000_000, 100_000_000) as i32;
    Point::new(clamp(mx + dy), clamp(my - dx))
}
/// Move a drawn item by `delta`.
pub(crate) fn move_graphic(graphic: &mut LayerGraphic, delta: Point) {
    match &mut graphic.shape {
        GraphicShape::Polyline { points } => points.iter_mut().for_each(|p| *p = p.offset(delta)),
        GraphicShape::Arc { start, mid, end } => {
            for p in [start, mid, end] {
                *p = p.offset(delta);
            }
        }
        GraphicShape::Circle { center, .. } => *center = center.offset(delta),
        GraphicShape::Text { at, .. } => *at = at.offset(delta),
    }
}
/// Whether `p` is inside the closed polyline `ring` (even-odd).
pub(crate) fn point_in(p: Point, ring: &[Point]) -> bool {
    let (x, y) = (f64::from(p.x), f64::from(p.y));
    let mut inside = false;
    for s in ring.windows(2) {
        let (a, b) = ((f64::from(s[0].x), f64::from(s[0].y)), (f64::from(s[1].x), f64::from(s[1].y)));
        if (a.1 > y) != (b.1 > y) && x < a.0 + (y - a.1) / (b.1 - a.1) * (b.0 - a.0) {
            inside = !inside;
        }
    }
    inside
}
/// [`action`] for the pads editor, so the table's closures need no type.
const fn pad_action(
    id: &'static str,
    label: &'static str,
    group: ActionGroup,
    run: fn(&mut FootprintEditor),
) -> Action<FootprintEditor> {
    action(id, label, group, run)
}
/// One step of the snapping grid as an action, pressed while it is the grid:
/// the host offers them as a menu, and a script picks one by id.
macro_rules! grid_action {
    ($id:literal, $label:literal, $grid:literal) => {
        pad_action($id, $label, ActionGroup::Grid, |e| e.set_grid($grid))
            .pressed_when(|e| e.grid == $grid)
    };
}
/// The tools that draw SILKSCREEN: decoration, on no pin and no net. The rest of
/// the Tool group draws copper.
const SILK_TOOLS: [&str; 5] =
    ["pads.tool.line", "pads.tool.rectangle", "pads.tool.circle", "pads.tool.arc", "pads.tool.text"];
/// Every command of the pads editor, in toolbar order.
pub fn pad_actions() -> &'static [Action<FootprintEditor>] {
    PAD_ACTIONS
}
static PAD_ACTIONS: &[Action<FootprintEditor>] = &[
    pad_action("pads.tool.select", "Select", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Select)
    })
    .keys(&[key(Key::Escape)])
    .pressed_when(|e| e.tool == DrawTool::Select),
    pad_action("pads.tool.smd", "SMD pad", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Pad)
    })
    .pressed_when(|e| e.tool == DrawTool::Pad),
    pad_action(
        "pads.tool.through_hole",
        "Through-hole pad",
        ActionGroup::Tool,
        |e| e.set_tool(DrawTool::ThroughHole),
    )
    .pressed_when(|e| e.tool == DrawTool::ThroughHole),
    pad_action("pads.tool.line", "Line", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Line)
    })
    .pressed_when(|e| e.tool == DrawTool::Line),
    pad_action("pads.tool.rectangle", "Rectangle", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Rectangle)
    })
    .pressed_when(|e| e.tool == DrawTool::Rectangle),
    pad_action("pads.tool.circle", "Circle", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Circle)
    })
    .pressed_when(|e| e.tool == DrawTool::Circle),
    pad_action("pads.tool.arc", "Arc", ActionGroup::Tool, |e| e.set_tool(DrawTool::Arc))
        .pressed_when(|e| e.tool == DrawTool::Arc),
    pad_action("pads.tool.text", "Text", ActionGroup::Tool, |e| e.set_tool(DrawTool::Text))
        .pressed_when(|e| e.tool == DrawTool::Text),
    pad_action(
        "pads.undo",
        "Undo",
        ActionGroup::History,
        FootprintEditor::undo,
    )
    .keys(&[command(Key::Z)])
    .enabled_when(|e| e.can_step_history(HistoryKey::Undo)),
    pad_action(
        "pads.redo",
        "Redo",
        ActionGroup::History,
        FootprintEditor::redo,
    )
    .keys(&[
        egui::KeyboardShortcut::new(
            egui::Modifiers::COMMAND.plus(egui::Modifiers::SHIFT),
            Key::Z,
        ),
        command(Key::Y),
    ])
    .enabled_when(|e| e.can_step_history(HistoryKey::Redo)),
    pad_action("pads.fit", "Fit", ActionGroup::Zoom, |e| e.fit = true).keys(&[key(Key::F)]),
    pad_action("pads.zoom_in", "Zoom +", ActionGroup::Zoom, |e| {
        e.core.zoom = crate::scaled_zoom(e.core.zoom, 1.25)
    }),
    pad_action("pads.zoom_out", "Zoom −", ActionGroup::Zoom, |e| {
        e.core.zoom = crate::scaled_zoom(e.core.zoom, 1.0 / 1.25)
    }),
    grid_action!("pads.grid.10", "Grid 0.01 mm", 10),
    grid_action!("pads.grid.50", "Grid 0.05 mm", 50),
    grid_action!("pads.grid.100", "Grid 0.1 mm", 100),
    grid_action!("pads.grid.250", "Grid 0.25 mm", 250),
    grid_action!("pads.grid.500", "Grid 0.5 mm", 500),
    grid_action!("pads.grid.635", "Grid 0.635 mm (25 mil)", 635),
    grid_action!("pads.grid.1270", "Grid 1.27 mm (50 mil)", 1270),
    grid_action!("pads.grid.2540", "Grid 2.54 mm (100 mil)", 2540),
    pad_action("pads.delete", "Delete item", ActionGroup::Selection, |e| {
        let before = e.footprint.clone();
        e.remove_selected();
        e.record(before);
    })
    .keys(&[key(Key::Delete), key(Key::Backspace)])
    .enabled_when(|e| e.core.selected.is_some()),
];

/// The pad a drawn outline would be, when the outline is one of the two shapes a
/// pad comes in: an axis-aligned RECTANGLE (as the rectangle tool draws one) or a
/// CIRCLE (as the circle tool draws one, as a closed polygon). Answers
/// `(centre, size, shape)`, or `None` for a line or any other run of points,
/// which is not a pad shape and is not guessed at.
fn silk_as_pad(line: &[Point]) -> Option<(Point, Point, PadShape)> {
    if line.len() < 4 {
        return None;
    }
    let (min, max) = (
        Point::new(
            line.iter().map(|p| p.x).min()?,
            line.iter().map(|p| p.y).min()?,
        ),
        Point::new(
            line.iter().map(|p| p.x).max()?,
            line.iter().map(|p| p.y).max()?,
        ),
    );
    let size = Point::new(max.x - min.x, max.y - min.y);
    if size.x <= 0 || size.y <= 0 {
        return None;
    }
    let at = Point::new(
        ((i64::from(min.x) + i64::from(max.x)) / 2) as i32,
        ((i64::from(min.y) + i64::from(max.y)) / 2) as i32,
    );
    // A tenth of the smaller side, so the test is on the drawn shape and not on
    // the rounding the tools left in it.
    let slack = f64::from(size.x.min(size.y)) / 10.;
    let on_the_box = line
        .iter()
        .all(|p| p.x == min.x || p.x == max.x || p.y == min.y || p.y == max.y);
    if on_the_box {
        return Some((at, size, PadShape::Rect));
    }
    let radius = f64::from(size.x.max(size.y)) / 2.;
    let round = line.iter().all(|p| {
        (f64::from(p.x - at.x).hypot(f64::from(p.y - at.y)) - radius).abs() <= slack
    });
    round.then(|| (at, Point::new(size.x, size.y), PadShape::Circle))
}
/// The widest corner radius, in screen points, egui draws on a rectangle: it
/// stores the radius in a byte.
pub(crate) const MAX_PAINTED_CORNER: f32 = 250.;
/// Whether the pad's copper side changed this frame: it was on the back and is
/// now on the front only, or the other way round.
fn pad_back_changed(was_back: bool, front_cu: bool, back_cu: bool) -> bool {
    (front_cu != back_cu) && (back_cu != was_back)
}
fn kind_name(kind: PadKind) -> &'static str {
    match kind {
        PadKind::SurfaceMount => "Surface mount",
        PadKind::ThroughHole => "Plated through hole",
        PadKind::Unplated => "Unplated hole",
    }
}
fn shape_name(shape: PadShape) -> &'static str {
    match shape {
        PadShape::Rect => "Rectangle",
        PadShape::RoundRect => "Rounded rectangle",
        PadShape::Circle => "Circle",
        PadShape::Oval => "Oval",
        PadShape::Trapezoid => "Trapezoid",
        PadShape::Custom => "Custom polygon",
    }
}
/// A custom pad's corners, each moved by its two fields, removed while more
/// than three remain, or added halfway along the edge that follows it, and the
/// rounding of its outline. The pad's size follows its corners.
fn custom_pad_fields(ui: &mut egui::Ui, hits: &mut InspectorHits, pad: &mut Pad) {
    ui.label("Corners").on_hover_text("Relative to the pad's position, which the copper must cover.");
    let n = pad.polygon.len();
    let (mut remove, mut add) = (None, None);
    for k in 0..n {
        ui.push_id(("corner", k), |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("{}", k + 1));
                let p = &mut pad.polygon[k];
                for (axis, v) in [("x", &mut p.x), ("y", &mut p.y)] {
                    let mut mm = f64::from(*v) / 1000.;
                    let field = ui.add(egui::DragValue::new(&mut mm).speed(0.01).range(-1000. ..=1000.).max_decimals(3).suffix(" mm"));
                    hits.mark(format!("corner:{}:{axis}", k + 1), &field);
                    if field.changed() {
                        *v = (mm * 1000.).round() as i32;
                    }
                }
                let insert = ui.small_button("+").on_hover_text("Add a corner halfway to the next");
                hits.mark(format!("corner:{}:add", k + 1), &insert);
                if insert.clicked() {
                    add = Some(k);
                }
                let delete = ui.add_enabled(n > 3, egui::Button::new("−").small()).on_hover_text("Remove this corner");
                hits.mark(format!("corner:{}:remove", k + 1), &delete);
                if delete.clicked() {
                    remove = Some(k);
                }
            });
        });
    }
    if let Some(k) = add {
        let (a, b) = (pad.polygon[k], pad.polygon[(k + 1) % n]);
        let mid = Point::new(((i64::from(a.x) + i64::from(b.x)) / 2) as i32, ((i64::from(a.y) + i64::from(b.y)) / 2) as i32);
        pad.polygon.insert(k + 1, mid);
    } else if let Some(k) = remove {
        pad.polygon.remove(k);
    }
    mm_field(ui, hits, "Rounding", &mut pad.corner_radius, 0. ..=100.);
    if let Some(problem) = pad.outline_problem() {
        ui.colored_label(ui.visuals().warn_fg_color, format!("This pad {problem}."));
    }
    if let (Some(lo), Some(hi)) = (
        pad.polygon.iter().copied().reduce(|a, b| Point::new(a.x.min(b.x), a.y.min(b.y))),
        pad.polygon.iter().copied().reduce(|a, b| Point::new(a.x.max(b.x), a.y.max(b.y))),
    ) {
        pad.size = Point::new((hi.x - lo.x).max(1), (hi.y - lo.y).max(1));
    }
}
/// A closed outline that is no rectangle or circle as a custom pad: its
/// position a point inside it and its corners about that point. `None` for an
/// open line or one that is not a simple polygon.
fn silk_as_custom_pad(line: &[Point]) -> Option<(Point, Vec<Point>)> {
    let (first, last) = (*line.first()?, *line.last()?);
    if line.len() < 4 || first != last {
        return None;
    }
    let ring = &line[..line.len() - 1];
    let at = brep_ecad_core::board::polygon_inner_point(ring);
    let mut pad = Pad::smd("", at, Point::new(1, 1), PadShape::Custom);
    pad.polygon = ring.iter().map(|p| Point::new(p.x - at.x, p.y - at.y)).collect();
    pad.outline_problem().is_none().then_some((at, pad.polygon))
}
/// The smallest copper gap between two pads the editor passes without a
/// warning: the board's default clearance, 0.2 mm, which the board's design
/// rule check applies to every pair of copper items of different nets. A
/// footprint is drawn before any board, so it is held to the default; a board
/// with a tighter rule still checks its own.
pub(crate) fn min_pad_gap() -> i32 {
    brep_ecad_core::board::DesignRules::default().clearance
}
fn mm_text(micrometres: i32) -> String {
    let text = format!("{:.3}", f64::from(micrometres) / 1000.);
    text.trim_end_matches('0').trim_end_matches('.').to_owned()
}
/// A point's two fields, keyed `<name>:x` and `<name>:y` ([`key_word`]).
fn point_fields(ui: &mut egui::Ui, hits: &mut InspectorHits, name: &str, p: &mut Point) {
    ui.push_id(name, |ui| {
        ui.label(name);
        let key = key_word(name);
        mm_field(ui, hits, &format!("{key}:x"), &mut p.x, -1000. ..=1000.);
        mm_field(ui, hits, &format!("{key}:y"), &mut p.y, -1000. ..=1000.);
    });
}
/// A length in millimetres, keyed by its name ([`key_word`]) unless the name
/// is already a key.
fn mm_field(
    ui: &mut egui::Ui,
    hits: &mut InspectorHits,
    name: &str,
    value: &mut i32,
    range: std::ops::RangeInclusive<f64>,
) {
    let mut mm = *value as f64 / 1000.;
    let (label, key) = match name.rsplit_once(':') {
        Some((_, axis)) => (axis.to_uppercase(), name.to_owned()),
        None => (name.to_owned(), key_word(name)),
    };
    ui.horizontal(|ui| {
        ui.label(label);
        let field = ui.add(
            egui::DragValue::new(&mut mm)
                .speed(0.01)
                .range(range)
                .max_decimals(3)
                .suffix(" mm"),
        );
        hits.mark(key, &field);
        if field.changed() {
            *value = (mm * 1000.).round() as i32;
        }
    });
}


/// The hit key of the pad at `index` of its footprint: `pad:<number>`, or, for
/// a MECHANICAL pad, whose number is blank, `unnumbered-pad:<index>` — the
/// twin of the symbol's `unnumbered-pin:`. A pad's number may be any other
/// text, so every key under `pad:` could be some pad's number, and `pad:` bare
/// is the family's own prefix: a mechanical pad published there was returned
/// by every `pad:` query, and two of them were one key. Out of the family, by
/// index — the one thing a mechanical pad still has. The Inspector's row is
/// `row:` and the same key.
fn pad_key(index: usize, pad: &Pad) -> String {
    if pad.number.trim().is_empty() {
        format!("unnumbered-pad:{index}")
    } else {
        format!("pad:{}", pad.number)
    }
}

/// Where the footprint point `p` is drawn on a canvas centred (with its pan)
/// at `center` — the one mapping the canvas paints with and
/// [`FootprintEditor::hits`] reports with.
fn on_canvas(center: Pos2, zoom: f32, p: Point) -> Pos2 {
    center + Vec2::new(p.x as f32, p.y as f32) * zoom
}
