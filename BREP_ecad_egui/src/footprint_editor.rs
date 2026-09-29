//! Footprint editor: pads, holes, and silkscreen, working like the symbol editor.
use super::*;
use crate::actions::{Action, ActionGroup, action, command, key};
use crate::board_view::{BACKGROUND, SILK, THROUGH_HOLE, WARNING, layer_color};
use crate::symbol_editor::{InspectorHits, LIST_SHARE, grip_under, key_word, list_height};
use brep_ecad_core::board::{COURTYARD_MARGIN, Footprint, Pad, PadShape, Shape};

#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum DrawTool {
    Select,
    Pad,
    ThroughHole,
    Line,
    Rectangle,
    Circle,
}
#[derive(Clone, Copy, PartialEq, Debug)]
enum Item {
    Pad(usize),
    Silk(usize),
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
    tool: DrawTool,
    selected: Option<Item>,
    start: Option<Point>,
    drag: Option<(Footprint, Point)>,
    history: History<Footprint>,
    zoom: f32,
    pan: Vec2,
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
    pending_change: Option<Change>,
    /// The Inspector's clickable widgets as it last drew them, for [`Self::hits`].
    inspector: InspectorHits,
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
            tool: DrawTool::Select,
            selected: None,
            start: None,
            drag: None,
            history: History::default(),
            zoom: 0.08,
            pan: Vec2::ZERO,
            fit: true,
            grid: 50,
            repeat: (1, 0, 1270),
            pending_number: None,
            canvas: Rect::from_min_size(Pos2::ZERO, Vec2::new(800., 600.)),
            error: String::new(),
            pending_change: None,
            inspector: InspectorHits::default(),
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
        if before != self.footprint {
            self.history.record(before, &self.footprint);
            self.note_change(coalesce.map(str::to_owned));
        }
    }
    fn note_change(&mut self, coalesce: Option<String>) {
        Change::record(&mut self.pending_change, coalesce);
    }
    /// The edits made since the last call, merged into one. A host that keeps the
    /// footprint in its own file calls this once a frame and stores [`Self::footprint`].
    pub fn take_change(&mut self) -> Option<Change> {
        self.pending_change.take()
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
        }
    }
    /// Where the canvas is looking: the footprint origin's offset from the canvas centre in
    /// points, and the zoom in points per micrometre. Read-only, for a host that shows it.
    pub fn pan_zoom(&self) -> (Vec2, f32) {
        (self.pan, self.zoom)
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
    /// [`SymbolEditor::selected_pin`] is; [`Item`] stays private.
    pub fn selected_pad(&self) -> Option<&str> {
        match self.selected {
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
                self.selected = Some(Item::Pad(index));
                // And bring it into view if it is off the canvas. A host's jump
                // from a connection point has to SHOW the pad, and the canvas may
                // be zoomed in on another corner of the footprint; a pad already on
                // screen does not move it.
                let at = self.footprint.pads[index].at;
                let centre = self.canvas.center() + self.pan;
                if !self
                    .canvas
                    .shrink(12.)
                    .contains(on_canvas(centre, self.zoom, at))
                {
                    self.pan = -Vec2::new(at.x as f32, at.y as f32) * self.zoom;
                }
                true
            }
            None => false,
        }
    }
    /// Where each pad is drawn, `pad:<number>` ([`pad_key`]: a mechanical
    /// pad's is `unnumbered-pad:<index>`), and each silkscreen line,
    /// `silk:<index>` (a small rect on the middle of its longest segment, as a
    /// sheet wire's is), in screen points on the `canvas` [`Self::show`] was
    /// given — the mapping the canvas paints with — and then the Inspector's
    /// widgets, `inspector:<thing>`, where [`Self::properties`] last drew them.
    pub fn hits(&self, canvas: Rect) -> Vec<(String, Rect)> {
        let center = canvas.center() + self.pan;
        let screen = |p: Point| on_canvas(center, self.zoom, p);
        let pads = self.footprint.pads.iter().enumerate().map(|(i, pad)| {
            let size = Vec2::new(pad.size.x.abs() as f32, pad.size.y.abs() as f32) * self.zoom;
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
        pads.chain(silk).chain(self.inspector.shown(self.canvas_pass)).collect()
    }
    /// The index of the selected silkscreen line, or `None` when the selection
    /// is a pad or empty — [`Self::selected_pad`]'s twin for silk, which has
    /// no number and is named by its place in the list, as `silk:<index>` is.
    pub fn selected_silk(&self) -> Option<usize> {
        match self.selected {
            Some(Item::Silk(index)) if index < self.footprint.silk.len() => Some(index),
            _ => None,
        }
    }
    /// Load a footprint the host holds, keeping zoom, pan, and grid. The editor's own
    /// undo history starts afresh and the load is not reported by [`Self::take_change`].
    pub fn set_footprint(&mut self, footprint: Footprint) {
        self.footprint = footprint;
        self.history = History::default();
        self.selected = None;
        self.drag = None;
        self.start = None;
        self.error.clear();
        self.pending_change = None;
    }
    /// Run a command from [`pad_actions`] by id, as a click on its button does.
    pub fn run_action(&mut self, id: &str) -> bool {
        crate::actions::run_action(self, pad_actions().iter(), id)
    }
    fn set_tool(&mut self, tool: DrawTool) {
        if let Some((before, _)) = self.drag.take() {
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
        if self.history.can_undo() {
            self.history.undo(&mut self.footprint);
            self.selected = None;
            self.note_change(None);
        }
    }
    /// Undo or redo for the keyboard or a host, unless a drag is still recording.
    pub(crate) fn step_history(&mut self, key: HistoryKey) {
        if self.drag.is_none() {
            match key {
                HistoryKey::Undo => self.undo(),
                HistoryKey::Redo => self.redo(),
            }
        }
    }
    pub(crate) fn can_step_history(&self, key: HistoryKey) -> bool {
        self.drag.is_none()
            && match key {
                HistoryKey::Undo => self.history.can_undo(),
                HistoryKey::Redo => self.history.can_redo(),
            }
    }
    fn redo(&mut self) {
        if self.history.can_redo() {
            self.history.redo(&mut self.footprint);
            self.selected = None;
            self.note_change(None);
        }
    }
    fn validate(&self) -> Result<(), String> {
        self.footprint.validate()
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
        self.drag = None;
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
        egui::ComboBox::from_id_salt(salt)
            .selected_text(format!("Grid {}", grid_text(self.grid)))
            .show_ui(ui, |ui| {
                for grid in GRIDS {
                    ui.selectable_value(&mut self.grid, grid, grid_text(grid));
                }
            })
            .response
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
                let gap = Shape::pad(a.at, a.size, a.shape).distance(&Shape::pad(b.at, b.size, b.shape));
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
    /// The editor in a window of its own, with Apply and Cancel for the standalone app.
    /// Returns Some(true) on Apply, Some(false) on Cancel or close.
    pub(crate) fn window(&mut self, ctx: &egui::Context) -> Option<bool> {
        let mut open = true;
        let mut action = None;
        // While this window is open the keyboard history is its own.
        self.run_shortcuts(ctx);
        egui::Window::new("Footprint editor")
            .id(egui::Id::new("footprint_editor"))
            .default_size(Vec2::new(980., 680.))
            .min_size(Vec2::new(660., 460.))
            .open(&mut open)
            .collapsible(false)
            .show(ctx, |ui| {
                self.toolbar(ui);
                ui.separator();
                ui.horizontal_top(|ui| {
                    let height = (ui.available_height() - 55.).max(300.);
                    ui.allocate_ui_with_layout(
                        Vec2::new(250., height),
                        egui::Layout::top_down(egui::Align::Min),
                        |ui| self.properties(ui),
                    );
                    let size = Vec2::new(ui.available_width().max(300.), height);
                    self.canvas(ui, size);
                });
                ui.separator();
                if !self.error.is_empty() {
                    ui.colored_label(ui.visuals().error_fg_color, &self.error);
                }
                ui.horizontal(|ui| {
                    if ui.button("Apply footprint").clicked() {
                        match self.validate() {
                            Ok(()) => action = Some(true),
                            Err(e) => self.error = e,
                        }
                    }
                    if ui.button("Cancel").clicked() {
                        action = Some(false);
                    }
                    ui.small(if self.target.is_some() {
                        "Updates this part and the session footprint library."
                    } else {
                        "Adds or updates this footprint in the session library."
                    });
                });
            });
        if !open {
            action = Some(false);
        }
        action
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
            "{} pads · {} silkscreen lines",
            self.footprint.pads.len(),
            self.footprint.silk.len()
        ));
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
            let grid = self.grid_combo(ui, "footprint-grid-inspector");
            self.inspector.mark("grid", &grid);
        });
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
        let reveal = self.selected != self.listed;
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
            .collect();
        // Read again: a pin's row just picked is a selection the Items list
        // has not shown.
        let reveal = self.selected != self.listed;
        let selected = self.selected;
        let mut pick = None;
        let keys: Vec<String> = self
            .footprint
            .pads
            .iter()
            .enumerate()
            .map(|(i, pad)| format!("row:{}", pad_key(i, pad)))
            .chain((0..self.footprint.silk.len()).map(|i| format!("row:silk:{i}")))
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
            self.selected = Some(item);
        }
        // A row clicked is on screen already: only a selection made elsewhere
        // scrolls the lists.
        self.listed = self.selected;
        ui.separator();
        ui.label("Item properties");
        match self.selected {
            Some(Item::Pad(i)) if i < self.footprint.pads.len() => self.pad_properties(ui, i),
            Some(Item::Silk(i)) if i < self.footprint.silk.len() => self.silk_properties(ui, i),
            _ => {
                ui.small("Select an item on the canvas or in the list.");
            }
        }
        if self.selected.is_some() {
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
        let selected = self.selected;
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
            self.selected = Some(Item::Pad(index));
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
        if let Some((at, size, shape)) = silk_as_pad(&self.footprint.silk[i]) {
            let number = self
                .missing_pins()
                .first()
                .map(|pin| (*pin).to_owned())
                .unwrap_or_else(|| self.next_number());
            let make = ui.button(format!("Make this a pad on pin {number}")).on_hover_text(
                "Replaces this outline with copper of the same size and place. Offered for a rectangle or a circle, which are the two shapes a pad comes in.",
            );
            self.inspector.mark("make_pad", &make);
            if make.clicked() {
                self.footprint.silk.remove(i);
                self.selected = Some(Item::Pad(self.footprint.pads.len()));
                self.footprint.pads.push(Pad {
                    number,
                    at,
                    size,
                    shape,
                    drill: None,
                    plated: true,
                });
                return;
            }
        }
        for (n, p) in self.footprint.silk[i].iter_mut().enumerate() {
            point_fields(ui, &mut self.inspector, &format!("Point {}", n + 1), p);
        }
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
        let choice = egui::ComboBox::from_label("Type")
            .selected_text(kind_name(kind))
            .show_ui(ui, |ui| {
                for k in [
                    PadKind::SurfaceMount,
                    PadKind::ThroughHole,
                    PadKind::Unplated,
                ] {
                    ui.selectable_value(&mut kind, k, kind_name(k));
                }
            });
        hits.mark("type", &choice.response);
        let default_drill = (pad.size.x.min(pad.size.y) * 6 / 10 / 50 * 50).max(100);
        match kind {
            PadKind::SurfaceMount => {
                pad.drill = None;
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
        let shape = egui::ComboBox::from_label("Shape")
            .selected_text(shape_name(pad.shape))
            .show_ui(ui, |ui| {
                for shape in [PadShape::Rect, PadShape::Circle, PadShape::Oval] {
                    ui.selectable_value(&mut pad.shape, shape, shape_name(shape));
                }
            });
        hits.mark("shape", &shape.response);
        point_fields(ui, hits, "Position", &mut pad.at);
        ui.label("Size");
        mm_field(ui, hits, "Width", &mut pad.size.x, 0.01..=100.);
        mm_field(ui, hits, "Height", &mut pad.size.y, 0.01..=100.);
        if let Some(drill) = &mut pad.drill {
            mm_field(ui, hits, "Drill", drill, 0.05..=100.);
        }
        let rotate = ui.button("Rotate pad 90°");
        hits.mark("rotate", &rotate);
        if rotate.clicked() {
            pad.size = Point::new(pad.size.y, pad.size.x);
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
        self.selected = Some(Item::Pad(self.footprint.pads.len() - 1));
    }
    fn remove_selected(&mut self) {
        match self.selected.take() {
            Some(Item::Pad(i)) if i < self.footprint.pads.len() => {
                self.footprint.pads.remove(i);
            }
            Some(Item::Silk(i)) if i < self.footprint.silk.len() => {
                self.footprint.silk.remove(i);
            }
            _ => {}
        }
    }
    fn hit(&self, p: Pos2, screen: impl Fn(Point) -> Pos2, world: Point) -> Option<Item> {
        let tolerance = f64::from(5. / self.zoom);
        for (i, pad) in self.footprint.pads.iter().enumerate().rev() {
            if Shape::pad(pad.at, pad.size, pad.shape).distance_to_point(world) <= tolerance {
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
            self.zoom = crate::valid_zoom(((rect.width() - 70.) / (max.x - min.x) as f32)
                .min((rect.height() - 70.) / (max.y - min.y) as f32)
                .min(1.), self.zoom);
            self.pan =
                -Vec2::new((min.x + max.x) as f32 / 2., (min.y + max.y) as f32 / 2.) * self.zoom;
            self.fit = false;
        }
        if response.dragged_by(PointerButton::Secondary)
            || response.dragged_by(PointerButton::Middle)
        {
            self.pan += ui.input(|i| i.pointer.delta());
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            self.zoom = crate::scaled_zoom(self.zoom, (scroll * 0.002).exp());
        }
        let zoom = self.zoom;
        let center = rect.center() + self.pan;
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
                self.selected = self.hit(p, screen, raw(p));
                self.drag = self.selected.map(|_| (self.footprint.clone(), raw(p)));
            } else {
                self.start = Some(self.snap(raw(p)));
            }
        }
        if let (Some((before, start)), Some(p)) = (&self.drag, pointer) {
            let q = raw(p);
            // Snap the offset so items that sit off the grid keep their position relative to it.
            let delta = self.snap(Point::new(q.x - start.x, q.y - start.y));
            self.footprint = before.clone();
            match self.selected {
                Some(Item::Pad(i)) => {
                    let pad = &mut self.footprint.pads[i];
                    pad.at = pad.at.offset(delta);
                }
                Some(Item::Silk(i)) => {
                    for p in &mut self.footprint.silk[i] {
                        *p = p.offset(delta);
                    }
                }
                None => {}
            }
        }
        if response.drag_stopped_by(PointerButton::Primary) {
            if let Some((before, _)) = self.drag.take() {
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
                DrawTool::Select => self.selected = self.hit(p, screen, raw(p)),
                _ => {}
            }
        }
        if response.secondary_clicked()
            && let Some(p) = pointer
        {
            self.selected = self.hit(p, screen, raw(p));
        }
        response.context_menu(|ui| {
            menu_ui(ui);
            if self.selected.is_some() && ui.button("Delete item").clicked() {
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
        if !self.footprint.pads.is_empty() || !self.footprint.silk.is_empty() {
            let (min, max) = self.footprint.bounds();
            let margin = Point::new(COURTYARD_MARGIN, COURTYARD_MARGIN);
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
            Shape::Rect { min, max } => {
                let r = Rect::from_two_pos(screen(min), screen(max));
                if let Some(fill) = fill {
                    painter.rect_filled(r, 0., fill);
                }
                if let Some(stroke) = outline {
                    painter.rect_stroke(r.expand(grow), 0., stroke, egui::StrokeKind::Outside);
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
            let shape = Shape::pad(pad.at, pad.size, pad.shape);
            let fill = match (pad.drill, pad.plated) {
                (None, _) => Some(layer_color(0, 2)),
                (Some(_), true) => Some(THROUGH_HOLE),
                (Some(_), false) => None,
            };
            paint_shape(shape, fill, None, 0.);
            if let Some(drill) = pad.drill {
                let radius = drill as f32 / 2. * zoom;
                painter.circle_filled(screen(pad.at), radius, BACKGROUND);
                if !pad.plated {
                    painter.circle_stroke(screen(pad.at), radius, Stroke::new(1., SILK));
                }
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
                paint_shape(shape, None, Some(Stroke::new(RING_WIDTH, colour)), RING_GAP);
            }
            if self.selected == Some(Item::Pad(i)) {
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
            let color = if self.selected == Some(Item::Silk(i)) {
                ACCENT
            } else {
                SILK
            };
            painter.add(egui::Shape::line(
                line.iter().map(|p| screen(*p)).collect(),
                Stroke::new(1.5, color),
            ));
        }
        // Hovering names the item under the pointer and says what it is on. The
        // canvas is where a user looks first, and a number drawn on a pad does not
        // say whether the symbol has a pin of that number.
        if self.drag.is_none()
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
            };
            response.clone().on_hover_text_at_pointer(note);
        }
        // Where a click would put the thing being drawn: the SNAPPED point, which
        // is not where the pointer is. At 50 µm a pointer at 1.27 mm put its pad
        // at 1.25, and nothing said so until the Inspector's Position field.
        if self.tool != DrawTool::Select
            && self.drag.is_none()
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
        if a == b && !pad_tool {
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
                self.selected = Some(Item::Pad(self.footprint.pads.len()));
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
                    plated: true,
                });
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
                self.selected = Some(Item::Silk(self.footprint.silk.len()));
                self.footprint.silk.push(line);
            }
            DrawTool::Select => {}
        }
        self.record(before);
    }
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
const SILK_TOOLS: [&str; 3] = ["pads.tool.line", "pads.tool.rectangle", "pads.tool.circle"];
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
        e.zoom = crate::scaled_zoom(e.zoom, 1.25)
    }),
    pad_action("pads.zoom_out", "Zoom −", ActionGroup::Zoom, |e| {
        e.zoom = crate::scaled_zoom(e.zoom, 1.0 / 1.25)
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
    .enabled_when(|e| e.selected.is_some()),
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
        PadShape::Circle => "Circle",
        PadShape::Oval => "Oval",
    }
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
