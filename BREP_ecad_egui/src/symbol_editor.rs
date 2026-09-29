use super::*;
use crate::actions::{Action, ActionGroup, action, command, key};
use brep_ecad_core::Pin;

/// The pitch every symbol coordinate is snapped to, in micrometres: 1.27 mm,
/// the 50 mil schematic grid a sheet's wires run on.
const GRID: i32 = 1270;
/// A pin's length when nothing else says: two grid steps, which is what a
/// schematic symbol's pins are drawn at very nearly everywhere.
const DEFAULT_PIN_LENGTH: i32 = 2 * GRID;
/// How near the pointer must come to a selected pin's end to take hold of it,
/// in screen points.
const HANDLE: f32 = 7.;

/// The direction a pin's TIP stands off its body end: which way the pin leaves
/// the symbol, and so which side of the body it belongs on.
///
/// There are four and nothing between them. A schematic pin is orthogonal by
/// every convention there is, every point on this canvas is snapped to
/// [`GRID`], and a tip that does not land on the grid is a tip no wire on a
/// sheet can be drawn to — so free rotation would buy an angle nobody draws at
/// the cost of pins that cannot be connected. The editor offers the four, and
/// makes each of them one drag or one keypress away.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Aim {
    Left,
    Right,
    Up,
    Down,
}
impl Aim {
    /// The four, horizontals first: the order the orientation buttons read in.
    const ALL: [Aim; 4] = [Aim::Left, Aim::Right, Aim::Up, Aim::Down];
    /// One micrometre this way. Y grows DOWNWARD, on the canvas and in the
    /// stored symbol alike, so Up is negative.
    const fn unit(self) -> (i32, i32) {
        match self {
            Aim::Left => (-1, 0),
            Aim::Right => (1, 0),
            Aim::Up => (0, -1),
            Aim::Down => (0, 1),
        }
    }
    const fn opposite(self) -> Aim {
        match self {
            Aim::Left => Aim::Right,
            Aim::Right => Aim::Left,
            Aim::Up => Aim::Down,
            Aim::Down => Aim::Up,
        }
    }
    /// A quarter turn clockwise on the canvas — the same turn
    /// [`Point::rotate(1)`](brep_ecad_core::Point::rotate) makes.
    const fn turned(self) -> Aim {
        match self {
            Aim::Right => Aim::Down,
            Aim::Down => Aim::Left,
            Aim::Left => Aim::Up,
            Aim::Up => Aim::Right,
        }
    }
    const fn label(self) -> &'static str {
        match self {
            Aim::Left => "Left",
            Aim::Right => "Right",
            Aim::Up => "Up",
            Aim::Down => "Down",
        }
    }
    /// The direction `delta` is nearest, a tie going to the horizontal — the
    /// rule the pin tool's own drag has always squared a direction by.
    fn of(delta: Point) -> Aim {
        if delta.x.abs() >= delta.y.abs() {
            if delta.x < 0 { Aim::Left } else { Aim::Right }
        } else if delta.y < 0 {
            Aim::Up
        } else {
            Aim::Down
        }
    }
}

/// `p` moved `length` micrometres in `aim`.
fn along(p: Point, aim: Aim, length: i32) -> Point {
    let (dx, dy) = aim.unit();
    Point::new(p.x + dx * length, p.y + dy * length)
}
/// `length` rounded to whole grid steps and never shorter than one: a tip off
/// the grid is a tip no sheet wire can reach, and a pin of no length has no
/// body to grab and no direction to read.
fn grid_length(length: i32) -> i32 {
    ((length as f64 / GRID as f64).round() as i32).max(1) * GRID
}
/// How far a pin's tip stands off its body end. For the axis-aligned pin every
/// gesture here makes, the other component is zero.
fn pin_length(pin: &Pin) -> i32 {
    (pin.at.x - pin.end.x)
        .abs()
        .max((pin.at.y - pin.end.y).abs())
}
/// Which way a pin points. A pin that sits on neither axis — one read from
/// elsewhere, or typed in by hand — reads as the nearest of the four, which is
/// where straightening it would put it; [`is_axial`] is what says it is not
/// straight yet.
fn pin_aim(pin: &Pin) -> Aim {
    Aim::of(Point::new(pin.at.x - pin.end.x, pin.at.y - pin.end.y))
}
/// Whether a pin lies along one axis with a length to it, which is the only
/// shape any gesture in this editor leaves one in.
fn is_axial(pin: &Pin) -> bool {
    (pin.at.x == pin.end.x) != (pin.at.y == pin.end.y)
}
/// Aim `pin` and set its length, holding its BODY END.
///
/// The body end is the end glued to the outline the user has already drawn, so
/// it is what every change of direction and of length turns and reaches from:
/// a longer pin reaches further OUT, and a turned pin swings its tip round the
/// body rather than dragging the body off the outline after it.
fn aim_pin(pin: &mut Pin, aim: Aim, length: i32) {
    pin.at = along(pin.end, aim, grid_length(length));
}

/// What a HOST's model says this symbol's pins bind to.
///
/// A part's symbol pins ARE its connection points, bound by NAME: the pin, the
/// pad and the point carry one name, and the point is minted for the pin that
/// names it. The join is the HOST's to make — this crate knows nothing of
/// ports blocks — so the host hands the editor the answer it already computes
/// and the editor shows each pin the point it is. The twin of
/// [`FootprintEditor::set_pins`](crate::FootprintEditor::set_pins), for the
/// other side of the same join.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PinPoints {
    /// The connection point each pin IS, as `(pin number, part-local address)`
    /// — `("1", "Pins.1")`. A pin with no entry here is the point of nothing.
    pub bound: Vec<(String, String)>,
    /// Everything the host says does not pair, IN ITS OWN WORDS, each tagged
    /// with the pin number it concerns when it concerns one. A tagged one is
    /// shown beside its pin, an untagged one in the summary; neither is ever
    /// reworded, so one problem reads the same here and on every other surface
    /// the host reports it on.
    pub problems: Vec<(Option<String>, String)>,
    /// Why the LATEST edit could not be carried to the points, if it could
    /// not. The live state of an edit in flight rather than a property of the
    /// symbol, and it belongs here because here is where that edit was typed.
    pub hold: Option<String>,
}

/// What the properties panel says about the connection point of the pin in
/// hand: see [`SymbolEditor::pin_binding`].
///
/// It carries its words rather than borrowing them, because the panel reads it
/// while it holds the pin itself mutably.
enum Binding {
    /// No host has said anything about points, so the panel says nothing. A
    /// standalone editor, and a part with no symbol block yet.
    Unknown,
    /// The point this pin is, by its part-local address.
    Point(String),
    /// It is the point of nothing, in the host's own words.
    Unbound(Vec<String>),
}

/// What a primary drag on the canvas has hold of.
#[derive(Clone, Copy, PartialEq)]
enum Grab {
    /// The item itself, which a drag anywhere on it moves.
    Move,
    /// The selected pin's TIP: aims and lengthens it, holding the body end.
    Tip,
    /// Its BODY END: the same, holding the tip. The electrical point stays
    /// where a sheet's wires already reach it and the body end swings, which is
    /// how a pin is moved onto another side of an outline already drawn.
    Root,
}

/// A primary drag in progress: the symbol as it was when the drag began — so
/// every frame re-applies the whole gesture to THAT, and one undo step undoes
/// the gesture — where the pointer went down, and what it took hold of.
struct Drag {
    before: Symbol,
    from: Point,
    grab: Grab,
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum DrawTool {
    Select,
    Line,
    Rectangle,
    Circle,
    Pin,
    Text,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Item {
    Graphic(usize),
    Pin(usize),
}

/// Draws one symbol: its outlines, text, and pins. The host places [`Self::toolbar`],
/// [`Self::properties`], and [`Self::show`], runs commands from [`symbol_actions`] by
/// id, and takes each edit with [`Self::take_change`].
pub struct SymbolEditor {
    pub symbol: Symbol,
    /// The component this was opened from, for the host to put the symbol back on.
    pub target: Option<Uuid>,
    tool: DrawTool,
    selected: Option<Item>,
    /// The selection the Items list last drew. When the two differ the
    /// selection came from elsewhere — the canvas, or Next unbound — and the
    /// list scrolls its row into view, so a pin chosen on a 40-pin symbol is
    /// never selected below the fold.
    listed: Option<Item>,
    start: Option<Point>,
    drag: Option<Drag>,
    /// The pin length the user last worked in, which the next pin drawn takes.
    /// One symbol's pins are all the same length in practice, so the editor
    /// remembers the answer rather than asking for it again.
    pin_length: i32,
    /// What the host says these pins bind to, or `None` while no host has
    /// said. `None` is NOT "nothing binds": an editor nobody has told says
    /// nothing at all about binding, and draws no warning for it.
    points: Option<PinPoints>,
    history: History<Symbol>,
    zoom: f32,
    pan: Vec2,
    fit: bool,
    error: String,
    pending_change: Option<Change>,
    /// The Inspector's clickable widgets as it last drew them, for [`Self::hits`].
    inspector: InspectorHits,
    /// The pass the canvas was last drawn in: the Inspector's rects are
    /// published only while it is drawn beside it.
    canvas_pass: u64,
    /// The Items list's height as the user dragged it, or `None` for the
    /// height it takes from the pane.
    list_height: Option<f32>,
}
impl SymbolEditor {
    pub fn new(symbol: Symbol, target: Option<Uuid>) -> Self {
        Self {
            symbol,
            target,
            tool: DrawTool::Select,
            selected: None,
            listed: None,
            start: None,
            drag: None,
            pin_length: DEFAULT_PIN_LENGTH,
            points: None,
            history: History::default(),
            zoom: 0.025,
            pan: Vec2::ZERO,
            fit: true,
            error: String::new(),
            pending_change: None,
            inspector: InspectorHits::default(),
            canvas_pass: 0,
            list_height: None,
        }
    }
    pub fn blank() -> Self {
        Self::new(
            Symbol {
                // Nameless until the user names it. A placed part's value is the
                // name's last `:` segment, so a stock name here would put that
                // stock word beside every placement; an empty one puts nothing.
                library_id: String::new(),
                reference_prefix: "U".into(),
                description: String::new(),
                unit_count: 1,
                properties: Default::default(),
                power_net: None,
                graphics: vec![],
                pins: vec![],
                graphic_gates: vec![],
                hide_pin_names: false,
                hide_pin_numbers: false,
            },
            None,
        )
    }
    fn record(&mut self, before: Symbol) {
        self.record_as(before, None);
    }
    /// Record a finished edit. Edits sharing `coalesce` are reported to the host as one
    /// change; the properties panel uses one key for all of its fields, so consecutive
    /// edits there merge whichever field they touched.
    fn record_as(&mut self, before: Symbol, coalesce: Option<&str>) {
        if before != self.symbol {
            self.history.record(before, &self.symbol);
            self.note_change(coalesce.map(str::to_owned));
        }
    }
    fn note_change(&mut self, coalesce: Option<String>) {
        Change::record(&mut self.pending_change, coalesce);
    }
    /// The edits made since the last call, merged into one. A host that keeps the symbol
    /// in its own file calls this once a frame and stores [`Self::symbol`].
    pub fn take_change(&mut self) -> Option<Change> {
        self.pending_change.take()
    }
    /// The drawing tool that is up, by the last word of the action that picks it
    /// (`symbol.tool.<name>`). Read-only, for a host that publishes it.
    pub fn tool_name(&self) -> &'static str {
        match self.tool {
            DrawTool::Select => "select",
            DrawTool::Line => "line",
            DrawTool::Rectangle => "rectangle",
            DrawTool::Circle => "circle",
            DrawTool::Pin => "pin",
            DrawTool::Text => "text",
        }
    }
    /// Where the canvas is looking: the symbol origin's offset from the canvas centre in
    /// points, and the zoom in points per micrometre. Read-only, for a host that shows it.
    pub fn pan_zoom(&self) -> (Vec2, f32) {
        (self.pan, self.zoom)
    }
    /// The NUMBER of the selected pin, or `None` when the selection is a graphic
    /// or empty. The identity a host joins on: a pin's number is what a pad
    /// matches and what a part's connection point is named, so this is the
    /// whole of the selection a host outside eCAD can act on. [`Item`] itself
    /// stays private — it indexes this editor's own lists.
    pub fn selected_pin(&self) -> Option<&str> {
        match self.selected {
            Some(Item::Pin(index)) => self.symbol.pins.get(index).map(|pin| pin.number.as_str()),
            _ => None,
        }
    }
    /// Select the pin numbered `number`, and say whether there was one. For a
    /// host driving the editor from outside — BREP's Qualify panel jumps here
    /// from a connection point of the same name. Not an edit: nothing is
    /// reported by [`Self::take_change`], because a selection is not a change
    /// to the symbol.
    pub fn select_pin(&mut self, number: &str) -> bool {
        match self.symbol.pins.iter().position(|pin| pin.number == number) {
            Some(index) => {
                self.selected = Some(Item::Pin(index));
                true
            }
            None => false,
        }
    }
    /// Tell the editor what this symbol's pins bind to, so each pin can show
    /// the connection point it is. `None` unsays it. For a host that keeps the
    /// symbol beside the part's connection points, as a part's blocks; it is
    /// not an edit, and nothing is reported by [`Self::take_change`].
    pub fn set_pin_points(&mut self, points: Option<PinPoints>) {
        self.points = points;
    }
    /// What the panel says about the point pin `number` is. `Unknown` while no
    /// host has said anything, which is the only state in which the editor
    /// keeps quiet about binding altogether.
    fn pin_binding(&self, number: &str) -> Binding {
        let Some(points) = &self.points else {
            return Binding::Unknown;
        };
        if let Some((_, address)) = points.bound.iter().find(|(pin, _)| pin == number) {
            return Binding::Point(address.clone());
        }
        let words: Vec<String> = points
            .problems
            .iter()
            .filter(|(pin, _)| pin.as_deref() == Some(number))
            .map(|(_, message)| message.clone())
            .collect();
        // The host names most of these itself. What is left is the case it
        // reports without naming a pin — a whole symbol unit that no port group
        // takes, whose pins are all unbound for the one reason — so the fallback
        // points at the summary rather than guessing at a reason of its own.
        Binding::Unbound(if words.is_empty() {
            vec!["not bound to a connection point; see Connection points below".to_owned()]
        } else {
            words
        })
    }
    /// Whether the host has said this pin is the point of nothing — which is
    /// false while no host has said anything at all. The canvas asks this of
    /// every pin every frame, so it reads the one list rather than going
    /// through [`Self::pin_binding`], which builds its words.
    fn unbound(&self, number: &str) -> bool {
        self.points
            .as_ref()
            .is_some_and(|points| !points.bound.iter().any(|(pin, _)| pin == number))
    }
    /// The indices of the pins that bind nothing, in the symbol's order.
    fn unbound_pins(&self) -> impl Iterator<Item = usize> + '_ {
        (0..self.symbol.pins.len()).filter(|&i| self.unbound(&self.symbol.pins[i].number))
    }
    /// Select the first unbound pin after the selected one, round to the
    /// start. Nothing selected, or a graphic, starts from the first pin.
    fn select_next_unbound(&mut self) {
        let after = match self.selected {
            Some(Item::Pin(i)) => i + 1,
            _ => 0,
        };
        let count = self.symbol.pins.len();
        if let Some(next) = (0..count)
            .map(|k| (after + k) % count)
            .find(|&i| self.unbound(&self.symbol.pins[i].number))
        {
            self.selected = Some(Item::Pin(next));
        }
    }
    /// Where each pin is drawn, `pin:<number>` ([`pin_key`]), in screen points
    /// on the `canvas` [`Self::show`] was given — the mapping the canvas paints
    /// with — and then the Inspector's widgets, `inspector:<thing>`, where
    /// [`Self::properties`] last drew them ([`InspectorHits`]).
    pub fn hits(&self, canvas: Rect) -> Vec<(String, Rect)> {
        let center = canvas.center() + self.pan;
        let screen = |p: Point| on_canvas(center, self.zoom, p);
        let mut hits: Vec<(String, Rect)> = self
            .symbol
            .pins
            .iter()
            .enumerate()
            .map(|(index, pin)| {
                let span = Rect::from_two_pos(screen(pin.at), screen(pin.end));
                (pin_key(index, pin), span.expand(4.))
            })
            .collect();
        // The SELECTED pin's two grab handles, where they are drawn and as far
        // as they actually reach ([`handle_reach`], so the published rect and
        // the hit test are one reading): a drag from `pin:<n>:tip` aims and
        // lengthens the pin from its body end, one from `pin:<n>:root` does it
        // from the tip. No pin selected, no handles, exactly as on screen.
        if let Some(Item::Pin(index)) = self.selected
            && let Some(pin) = self.symbol.pins.get(index)
        {
            let (tip, root) = (screen(pin.at), screen(pin.end));
            let size = Vec2::splat(2. * handle_reach(tip, root));
            let key = pin_key(index, pin);
            hits.push((format!("{key}:tip"), Rect::from_center_size(tip, size)));
            hits.push((format!("{key}:root"), Rect::from_center_size(root, size)));
        }
        hits.extend(self.inspector.shown(self.canvas_pass));
        hits
    }
    /// Load a symbol the host holds, keeping zoom and pan. The editor's own undo history
    /// starts afresh and the load is not reported by [`Self::take_change`].
    pub fn set_symbol(&mut self, symbol: Symbol) {
        self.symbol = symbol;
        self.history = History::default();
        self.selected = None;
        self.drag = None;
        self.start = None;
        self.error.clear();
        self.pending_change = None;
    }
    /// Bring a multi-unit symbol the KiCad import wrote before gates existed up to
    /// the gate shape ([`Symbol::migrate_legacy_units`]), reported to the host as a
    /// change outside this editor's own undo. Returns whether it changed anything.
    /// A host calls it on the frame it first loads the stored symbol, for the reason
    /// [`crate::Editor::adopt_legacy_units`] gives.
    pub fn adopt_legacy_units(&mut self) -> bool {
        let migrated = self.symbol.migrate_legacy_units();
        if migrated {
            self.note_change(None);
        }
        migrated
    }
    fn set_tool(&mut self, tool: DrawTool) {
        if let Some(drag) = self.drag.take() {
            self.symbol = drag.before;
        }
        self.tool = tool;
        self.start = None;
    }
    /// Run a command from [`symbol_actions`] by id, as a click on its button does.
    pub fn run_action(&mut self, id: &str) -> bool {
        crate::actions::run_action(self, symbol_actions().iter(), id)
    }
    fn undo(&mut self) {
        if self.history.can_undo() {
            self.history.undo(&mut self.symbol);
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
            self.history.redo(&mut self.symbol);
            self.selected = None;
            self.note_change(None);
        }
    }
    fn validate(&self) -> Result<(), String> {
        if self.symbol.library_id.trim().is_empty()
            || self.symbol.reference_prefix.trim().is_empty()
        {
            return Err("Enter a symbol name and reference prefix.".into());
        }
        let mut d = Document::default();
        d.place(self.symbol.clone(), Point::new(0, 0), 0);
        d.validate()
    }
    /// The tool buttons, history, and zoom, with a line on the tool in use.
    pub fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            for group in [ActionGroup::Tool, ActionGroup::History, ActionGroup::Zoom] {
                crate::actions::action_buttons(self, symbol_actions().iter(), ui, group);
            }
        });
        ui.label(match self.tool {
            DrawTool::Select => {
                "Select or drag an item. Drag a selected pin's ends to aim and lengthen it, press R to turn it, or [ and ] to shorten and lengthen it. Right-drag pans. Scroll zooms. Grid: 1.27 mm."
            }
            DrawTool::Pin => {
                "Drag from the electrical connection tip toward the symbol body to add a pin, or click to place one of the last length used."
            }
            DrawTool::Text => "Click to add text, then edit it in Item properties.",
            _ => "Drag to draw. Use Item properties to adjust coordinates in millimetres.",
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
            let actions: Vec<_> = symbol_actions().iter().collect();
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
        egui::Window::new("Symbol editor")
            .id(egui::Id::new("symbol_editor"))
            .default_size(Vec2::new(960., 660.))
            .min_size(Vec2::new(640., 450.))
            .open(&mut open)
            .collapsible(false)
            .show(ctx, |ui| {
                self.toolbar(ui);
                ui.separator();
                ui.horizontal_top(|ui| {
                    ui.allocate_ui_with_layout(
                        Vec2::new(220., (ui.available_height() - 55.).max(300.)),
                        egui::Layout::top_down(egui::Align::Min),
                        |ui| self.properties(ui),
                    );
                    let size = Vec2::new(
                        ui.available_width().max(300.),
                        (ui.available_height() - 55.).max(300.),
                    );
                    self.canvas(ui, size);
                });
                ui.separator();
                if !self.error.is_empty() {
                    ui.colored_label(ui.visuals().error_fg_color, &self.error);
                }
                ui.horizontal(|ui| {
                    if ui.button("Apply symbol").clicked() {
                        match self.validate() {
                            Ok(()) => action = Some(true),
                            Err(e) => self.error = e,
                        }
                    }
                    if ui.button("Cancel").clicked() {
                        action = Some(false);
                    }
                    ui.small(if self.target.is_some() {
                        "Updates this component and the session library."
                    } else {
                        "Adds or updates this symbol in the session library."
                    });
                });
            });
        if !open {
            action = Some(false);
        }
        action
    }
    /// The symbol's name and the selected item's fields: the Inspector. It
    /// scrolls as a whole, so nothing at its foot is lost below a short pane,
    /// and it records where each of its widgets is for [`Self::hits`].
    pub fn properties(&mut self, ui: &mut egui::Ui) {
        // The pane's own height, read before the scroll area makes it endless:
        // the Items list takes its share of it.
        let pane = ui.available_height();
        self.inspector.begin(ui.ctx());
        egui::ScrollArea::vertical()
            .id_salt("symbol_inspector")
            .auto_shrink([false, true])
            .show(ui, |ui| self.inspector_body(ui, pane));
    }
    fn inspector_body(&mut self, ui: &mut egui::Ui, pane: f32) {
        let before = self.symbol.clone();
        // Typing in a field reports one change per keystroke under one key, so
        // a run of them reaches the host as one undo step. A BUTTON here is a
        // gesture of its own and must not merge with the typing around it, so
        // a press drops the key for this frame.
        let mut coalesce = Some("symbol:properties");
        let hits = &mut self.inspector;
        ui.label("Symbol name");
        hits.mark(
            "symbol_name",
            &ui.add(
                egui::TextEdit::singleline(&mut self.symbol.library_id)
                    .hint_text("Library:Name — the name is a placed part's value"),
            ),
        );
        ui.label("Reference prefix");
        hits.mark("reference_prefix", &ui.text_edit_singleline(&mut self.symbol.reference_prefix));
        ui.label("Description");
        hits.mark("description", &ui.text_edit_multiline(&mut self.symbol.description));
        // KiCad's `(pin_names (hide yes))` and `(pin_numbers (hide yes))`: what
        // the SHEET prints beside each pin. This canvas shows both regardless,
        // since here they are what is being edited.
        let mut names = !self.symbol.hide_pin_names;
        let mut numbers = !self.symbol.hide_pin_numbers;
        ui.horizontal_wrapped(|ui| {
            ui.label("On the sheet, show pin");
            hits.mark("show_pin_names", &ui.checkbox(&mut names, "names"));
            hits.mark("show_pin_numbers", &ui.checkbox(&mut numbers, "numbers"));
        });
        self.symbol.hide_pin_names = !names;
        self.symbol.hide_pin_numbers = !numbers;
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Items");
            ui.small(format!(
                "{} pins, {} graphics",
                self.symbol.pins.len(),
                self.symbol.graphics.len()
            ));
        });
        // Which rows to mark, read before the list takes the selection
        // mutably. A pin that is the point of nothing says so HERE too: the
        // summary below counts them without naming them, and the canvas marks
        // only a ring at each tip, so with several unbound the list is the one
        // place that says WHICH without a click on each.
        let unbound: Vec<bool> = self.symbol.pins.iter().map(|p| self.unbound(&p.number)).collect();
        // A long symbol's list scrolls, so the unbound pins are NAMED above
        // it, where no scrolling can hide them, with the way to each of them.
        let names: Vec<String> = self
            .unbound_pins()
            .map(|i| match self.symbol.pins[i].number.trim() {
                "" => "(no number)".to_owned(),
                number => number.to_owned(),
            })
            .collect();
        if !names.is_empty() {
            const NAMED: usize = 8;
            let mut line = names.iter().take(NAMED).cloned().collect::<Vec<_>>().join(", ");
            if names.len() > NAMED {
                line += &format!(" and {} more", names.len() - NAMED);
            }
            let noun = if names.len() == 1 { "pin binds" } else { "pins bind" };
            ui.label(
                egui::RichText::new(format!("{} {noun} nothing: {line}", names.len()))
                    .color(ui.visuals().warn_fg_color),
            );
            let next = ui
                .button("Next unbound pin")
                .on_hover_text("Selects the next pin that binds nothing, and shows its row. U, with the pointer over the canvas.");
            self.inspector.mark("next_unbound", &next);
            if next.clicked() {
                self.select_next_unbound();
            }
        }
        // The selection came from elsewhere since the list was last drawn.
        let reveal = self.selected != self.listed;
        // A share of the pane, never less than the nine rows it always had, or
        // the height the user dragged it to.
        let height = list_height(self.list_height, pane, LIST_SHARE, LIST_HEIGHT);
        let hits = &mut self.inspector;
        let list = egui::ScrollArea::vertical()
            .id_salt("symbol_items")
            .max_height(height)
            .show(ui, |ui| {
                for (i, g) in self.symbol.graphics.iter().enumerate() {
                    let name = match g {
                        Graphic::Path(_) => "Outline",
                        Graphic::Circle { .. } => "Circle",
                        Graphic::Text { .. } => "Text",
                    };
                    let response = ui.selectable_label(
                        self.selected == Some(Item::Graphic(i)),
                        format!("{name} {}", i + 1),
                    );
                    if reveal && self.selected == Some(Item::Graphic(i)) {
                        response.scroll_to_me(Some(egui::Align::Center));
                    }
                    hits.mark(format!("row:graphic:{i}"), &response);
                    if response.clicked() {
                        self.selected = Some(Item::Graphic(i));
                    }
                }
                for (i, p) in self.symbol.pins.iter().enumerate() {
                    let number = if p.number.trim().is_empty() {
                        "(no number)"
                    } else {
                        p.number.as_str()
                    };
                    let row = match crate::pin_name(&p.name) {
                        Some(name) => format!("Pin {number} · {name}"),
                        None => format!("Pin {number}"),
                    };
                    // The theme's warning colour, not the canvas's: the list is
                    // drawn on the theme's ground. Not on the selected row, whose
                    // own fill it does not read on; the Inspector below says it
                    // for that pin.
                    let row = if unbound[i] && self.selected != Some(Item::Pin(i)) {
                        egui::RichText::new(row).color(ui.visuals().warn_fg_color)
                    } else {
                        egui::RichText::new(row)
                    };
                    let response =
                        ui.selectable_label(self.selected == Some(Item::Pin(i)), row);
                    if reveal && self.selected == Some(Item::Pin(i)) {
                        response.scroll_to_me(Some(egui::Align::Center));
                    }
                    // Named as the pin is on the canvas, so a script reaches
                    // one pin by one name in both places.
                    hits.mark(format!("row:{}", pin_key(i, p)), &response);
                    let response = if unbound[i] {
                        response.on_hover_text("Not bound to a connection point. Select it to see why.")
                    } else {
                        response
                    };
                    if response.clicked() {
                        self.selected = Some(Item::Pin(i));
                    }
                }
            });
        if let Some(grip) = grip_under(ui, &list, &mut self.list_height) {
            self.inspector.mark("items_grip", &grip);
        }
        // A row clicked is on screen already: only a selection made elsewhere
        // scrolls the list.
        self.listed = self.selected;
        ui.separator();
        ui.label("Item properties");
        // Set by a button that must reach past the borrow of the pin it was
        // pressed under, so it is carried out below.
        let mut level: Option<i32> = None;
        match self.selected {
            Some(Item::Pin(i)) if i < self.symbol.pins.len() => {
                // Read what the panel must SAY about this pin before taking the
                // pin itself mutably: the binding lives beside the symbol, not
                // in it.
                let number = self.symbol.pins[i].number.clone();
                let binding = self.pin_binding(&number);
                // A number is what names the pin's connection point and what
                // its pad matches, so a pin with none, or with one another pin
                // has, binds to nothing. The field is NOT refused: a number is
                // cleared on the way to typing a new one, and refusing that
                // keystroke is what the host's hold exists to avoid. The fix
                // is OFFERED instead, one press away.
                let clash = if number.trim().is_empty() {
                    Some("No number. A pin's number names its connection point and the pad it lands on.".to_owned())
                } else if self.symbol.pins.iter().filter(|q| q.number == number).count() > 1 {
                    Some(format!("Another pin is numbered {number} too, so neither is one point."))
                } else {
                    None
                };
                let free = self.free_number();
                let p = &mut self.symbol.pins[i];
                let hits = &mut self.inspector;
                ui.label("Pin number");
                hits.mark("pin_number", &ui.text_edit_singleline(&mut p.number));
                if let Some(why) = clash {
                    // Body size: a warning is the line on this panel most
                    // worth reading, and `small` made it the hardest.
                    ui.label(egui::RichText::new(why).color(ui.visuals().warn_fg_color));
                    let offer = ui
                        .button(format!("Number it {free}"))
                        .on_hover_text("The lowest number no other pin of this symbol has.");
                    hits.mark("number_it", &offer);
                    if offer.clicked() {
                        p.number = free;
                        coalesce = None;
                    }
                }
                ui.label("Pin name");
                hits.mark("pin_name", &ui.text_edit_singleline(&mut p.name));
                // What this pin IS on the part: the connection point of its
                // number, which the pads and the 3D model name the same way.
                // Said HERE, beside the number it is named by, so that authoring
                // a pin and declaring a point are visibly the one act they are.
                match &binding {
                    Binding::Unknown => {}
                    Binding::Point(address) => {
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Connection point");
                            ui.label(egui::RichText::new(address).monospace().color(ACCENT))
                                .on_hover_text(POINT_HINT);
                        });
                    }
                    Binding::Unbound(words) => {
                        ui.label(
                            egui::RichText::new("Connection point: none")
                                .color(ui.visuals().warn_fg_color),
                        );
                        for line in words {
                            ui.label(egui::RichText::new(line).color(ui.visuals().warn_fg_color));
                        }
                    }
                }
                let kind = egui::ComboBox::from_label("Electrical type")
                    .selected_text(&p.electrical_type)
                    .show_ui(ui, |ui| {
                        for t in [
                            "passive",
                            "input",
                            "output",
                            "bidirectional",
                            "tri_state",
                            "power_in",
                            "power_out",
                            "open_collector",
                            "open_emitter",
                            "no_connect",
                            "unspecified",
                        ] {
                            ui.selectable_value(&mut p.electrical_type, t.into(), t);
                        }
                    });
                hits.mark("electrical_type", &kind.response);
                hits.mark("hidden", &ui.checkbox(&mut p.hidden, "Hidden"));
                // Which way the pin leaves the body, and how far it reaches.
                // Both are the vector from `end` to `at` read two ways, and
                // both hold the BODY END: the outline the user has already
                // drawn does not move because a pin on it was turned.
                let (aim, length, axial) = (pin_aim(p), pin_length(p).max(GRID), is_axial(p));
                ui.label("Orientation");
                ui.horizontal_wrapped(|ui| {
                    for choice in Aim::ALL {
                        let button =
                            egui::Button::selectable(axial && choice == aim, choice.label());
                        let button = ui.add(button).on_hover_text(TURN_HINT);
                        hits.mark(format!("aim:{}", choice.label().to_lowercase()), &button);
                        if button.clicked() {
                            aim_pin(p, choice, length);
                            coalesce = None;
                        }
                    }
                });
                if !axial {
                    ui.label(
                        egui::RichText::new("On neither axis. Pick a direction to straighten it.")
                            .color(ui.visuals().warn_fg_color),
                    );
                }
                ui.horizontal(|ui| {
                    ui.label("Length");
                    let mut mm = length as f64 / 1000.;
                    let step = GRID as f64 / 1000.;
                    let field = egui::DragValue::new(&mut mm)
                        .speed(step)
                        .range(step..=127.)
                        .suffix(" mm");
                    let field = ui.add(field);
                    hits.mark("length", &field);
                    if field.changed() {
                        aim_pin(p, aim, (mm * 1000.).round() as i32);
                    }
                });
                // The pin in hand is the length the user is working in, so the
                // next pin drawn takes it — whether it was set here or simply
                // selected.
                self.pin_length = pin_length(p).max(GRID);
                let same = ui.button("Same length for every pin").on_hover_text(
                    "Reaches every pin out this far from where it meets the body, each keeping the direction it has.",
                );
                hits.mark("same_length", &same);
                if same.clicked() {
                    level = Some(self.pin_length);
                }
                point_fields(ui, hits, "Tip", &mut p.at);
                point_fields(ui, hits, "Body end", &mut p.end);
            }
            Some(Item::Graphic(i)) if i < self.symbol.graphics.len() => {
                let hits = &mut self.inspector;
                match &mut self.symbol.graphics[i] {
                    Graphic::Path(points) => {
                        for (n, p) in points.iter_mut().enumerate() {
                            point_fields(ui, hits, &format!("Point {}", n + 1), p);
                        }
                    }
                    Graphic::Circle { center, radius } => {
                        point_fields(ui, hits, "Center", center);
                        mm_field(ui, hits, "Radius", radius, 0.001..=100000.);
                    }
                    Graphic::Text { at, text, size } => {
                        hits.mark("text", &ui.text_edit_multiline(text));
                        point_fields(ui, hits, "Position", at);
                        mm_field(ui, hits, "Text size", size, 0.1..=100.);
                    }
                }
            }
            _ => {
                ui.small("Nothing selected.");
                ui.small(match self.tool {
                    DrawTool::Select => "Click an item on the canvas, or a row under Items.",
                    DrawTool::Pin => {
                        "Drag from the tip toward the body to add a pin, or click to place one."
                    }
                    DrawTool::Text => "Click on the canvas to add text.",
                    _ => "Drag on the canvas to draw.",
                });
            }
        }
        if let Some(length) = level {
            for pin in &mut self.symbol.pins {
                let aim = pin_aim(pin);
                aim_pin(pin, aim, length);
            }
            coalesce = None;
        }
        if self.selected.is_some() {
            let delete = ui.button("Delete item");
            self.inspector.mark("delete", &delete);
            if delete.clicked() {
                self.remove_selected();
                coalesce = None;
            }
        }
        self.record_as(before, coalesce);
        self.connection_points(ui);
    }
    /// How this symbol's pins and the part's connection points stand, in the
    /// HOST's own words — the part of its report that concerns no one pin, and
    /// the live hold. Nothing at all while no host has said anything.
    fn connection_points(&self, ui: &mut egui::Ui) {
        let Some(points) = &self.points else {
            return;
        };
        ui.separator();
        ui.label("Connection points");
        let pins = self.symbol.pins.len();
        let summary = format!("{} of {pins} pins bound", points.bound.len());
        // Body size, the warnings above all: they are what this section is
        // for, and at `small` they were the hardest words on the panel to read.
        if points.problems.is_empty() && points.bound.len() == pins {
            ui.label(summary);
        } else {
            ui.label(egui::RichText::new(summary).color(ui.visuals().warn_fg_color));
        }
        for (_, message) in points.problems.iter().filter(|(pin, _)| pin.is_none()) {
            ui.label(egui::RichText::new(message).color(ui.visuals().warn_fg_color));
        }
        if let Some(hold) = &points.hold {
            ui.label(
                egui::RichText::new(format!("not carried to the points: {hold}"))
                    .color(ui.visuals().warn_fg_color),
            );
        }
    }
    /// Turn the selected pin a quarter turn clockwise about its BODY END, so
    /// the end glued to the outline stays put and the tip swings round it. One
    /// undo step per press, however many presses go round.
    fn rotate_pin(&mut self) {
        let Some(Item::Pin(index)) = self.selected else {
            return;
        };
        let before = self.symbol.clone();
        if let Some(pin) = self.symbol.pins.get_mut(index) {
            let (aim, length) = (pin_aim(pin), pin_length(pin).max(GRID));
            aim_pin(pin, aim.turned(), length);
        }
        self.record(before);
    }
    /// Reach the selected pin one grid step further out (`steps` > 0) or back
    /// in (`steps` < 0), holding its BODY END and its direction, never shorter
    /// than one step. The keyboard's half of the `Length` field: the field
    /// takes any length, these take the one a schematic pin is changed by. One
    /// undo step per press, and the next pin drawn takes the new length.
    fn step_pin_length(&mut self, steps: i32) {
        let Some(Item::Pin(index)) = self.selected else {
            return;
        };
        let before = self.symbol.clone();
        if let Some(pin) = self.symbol.pins.get_mut(index) {
            let (aim, length) = (pin_aim(pin), pin_length(pin).max(GRID));
            aim_pin(pin, aim, length + steps * GRID);
            self.pin_length = pin_length(pin);
        }
        self.record(before);
    }
    /// Whether the selected pin can be shortened: it is longer than one step.
    fn pin_can_shorten(&self) -> bool {
        matches!(self.selected, Some(Item::Pin(index))
            if self.symbol.pins.get(index).is_some_and(|pin| pin_length(pin) > GRID))
    }
    /// The lowest whole number no pin of this symbol carries: what a new pin
    /// is numbered, and what a pin with no number, or one it shares, is
    /// OFFERED.
    fn free_number(&self) -> String {
        (1..)
            .map(|n: u32| n.to_string())
            .find(|n| !self.symbol.pins.iter().any(|p| p.number == *n))
            .unwrap()
    }
    /// Whether the selected item is a pin this symbol still has.
    fn pin_selected(&self) -> bool {
        matches!(self.selected, Some(Item::Pin(index)) if index < self.symbol.pins.len())
    }
    /// The gate a symbol split into gates draws what is added at `p` with: the
    /// one whose box (grown by 2.54 mm) holds it, else the nearest. `0` for a
    /// symbol drawn whole.
    fn gate_at(&self, p: Point) -> u32 {
        let count = self.symbol.gate_count();
        if count <= 1 {
            return 0;
        }
        (1..=count)
            .filter_map(|gate| {
                let (lo, hi) = self.symbol.gate_extent(gate)?;
                let dx = (lo.x - 2540 - p.x).max(p.x - hi.x - 2540).max(0);
                let dy = (lo.y - 2540 - p.y).max(p.y - hi.y - 2540).max(0);
                Some((i64::from(dx).pow(2) + i64::from(dy).pow(2), gate))
            })
            .min()
            .map_or(0, |(_, gate)| gate)
    }
    fn remove_selected(&mut self) {
        match self.selected.take() {
            Some(Item::Graphic(i)) if i < self.symbol.graphics.len() => {
                self.symbol.graphics.remove(i);
                if i < self.symbol.graphic_gates.len() {
                    self.symbol.graphic_gates.remove(i);
                }
            }
            Some(Item::Pin(i)) if i < self.symbol.pins.len() => {
                self.symbol.pins.remove(i);
            }
            _ => {}
        }
    }
    /// The handle of the SELECTED pin under `p`, if the pointer is on one.
    /// Asked before the general hit test: a handle sits on top of the pin's own
    /// body, and taking hold of one is what aims the pin rather than moving it.
    fn grab_at(&self, p: Pos2, screen: impl Fn(Point) -> Pos2) -> Option<Grab> {
        let Some(Item::Pin(index)) = self.selected else {
            return None;
        };
        let pin = self.symbol.pins.get(index)?;
        let (tip, root) = (screen(pin.at), screen(pin.end));
        let reach = handle_reach(tip, root);
        [(Grab::Tip, tip), (Grab::Root, root)]
            .into_iter()
            .find(|(_, at)| p.distance(*at) <= reach)
            .map(|(grab, _)| grab)
    }
    fn hit(&self, p: Pos2, screen: impl Fn(Point) -> Pos2) -> Option<Item> {
        for (i, pin) in self.symbol.pins.iter().enumerate().rev() {
            if segment_distance(p, screen(pin.at), screen(pin.end)) < 9. {
                return Some(Item::Pin(i));
            }
        }
        for (i, g) in self.symbol.graphics.iter().enumerate().rev() {
            let hit = match g {
                Graphic::Path(points) => points
                    .windows(2)
                    .any(|a| segment_distance(p, screen(a[0]), screen(a[1])) < 8.),
                Graphic::Circle { center, radius } => {
                    (p.distance(screen(*center)) - *radius as f32 * self.zoom).abs() < 8.
                }
                Graphic::Text { at, text, size } => Rect::from_min_size(
                    screen(*at),
                    Vec2::new(
                        text.len() as f32 * *size as f32 * self.zoom * 0.65,
                        *size as f32 * self.zoom,
                    ),
                )
                .expand(5.)
                .contains(p),
            };
            if hit {
                return Some(Item::Graphic(i));
            }
        }
        None
    }
    fn canvas(&mut self, ui: &mut egui::Ui, size: Vec2) {
        self.canvas_pass = ui.ctx().cumulative_pass_nr();
        let (response, painter) = ui.allocate_painter(size, Sense::click_and_drag());
        let rect = response.rect;
        if self.fit {
            let mut points = vec![Point::new(-5080, -5080), Point::new(5080, 5080)];
            for g in &self.symbol.graphics {
                match g {
                    Graphic::Path(p) => points.extend(p),
                    Graphic::Circle { center, radius } => {
                        points.push(center.offset(Point::new(*radius, *radius)));
                        points.push(center.offset(Point::new(-radius, -radius)));
                    }
                    Graphic::Text { at, .. } => points.push(*at),
                }
            }
            points.extend(self.symbol.pins.iter().flat_map(|p| [p.at, p.end]));
            let min = Point::new(
                points.iter().map(|p| p.x).min().unwrap(),
                points.iter().map(|p| p.y).min().unwrap(),
            );
            let max = Point::new(
                points.iter().map(|p| p.x).max().unwrap(),
                points.iter().map(|p| p.y).max().unwrap(),
            );
            // A symbol split into gates has a caption centred over each, and the
            // outermost one must not run off the canvas (the re-audit's cut-off
            // "Unit 3" of the LM358).
            // It also stands some 40 points over its gate, so it needs room above.
            let (captions, over) = if self.symbol.gate_count() > 1 { (140., 80.) } else { (0., 0.) };
            self.zoom = crate::valid_zoom(((rect.width() - 70. - captions) / (max.x - min.x) as f32)
                .min((rect.height() - 70. - over) / (max.y - min.y) as f32)
                .min(0.06), self.zoom);
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
        let world = |p: Pos2| {
            Point::new(
                ((p.x - center.x) / zoom).round() as i32,
                ((p.y - center.y) / zoom).round() as i32,
            )
            .snapped()
        };
        let pointer = response.interact_pointer_pos();
        if response.drag_started_by(PointerButton::Primary)
            && let Some(p) = ui.input(|i| i.pointer.press_origin())
        {
            if self.tool == DrawTool::Select {
                // A handle of the pin ALREADY selected comes first: it lies on
                // the pin's own body, so hit-testing the body first would move
                // the pin every time the user reached for its end.
                let grab = self.grab_at(p, screen);
                if grab.is_none() {
                    self.selected = self.hit(p, screen);
                }
                self.drag = (grab.is_some() || self.selected.is_some()).then(|| Drag {
                    before: self.symbol.clone(),
                    from: world(p),
                    grab: grab.unwrap_or(Grab::Move),
                });
            } else {
                self.start = Some(world(p));
            }
        }
        if let (Some(drag), Some(p)) = (&self.drag, pointer) {
            let (from, grab) = (drag.from, drag.grab);
            self.symbol = drag.before.clone();
            let q = world(p);
            let delta = Point::new(q.x - from.x, q.y - from.y);
            match (grab, self.selected) {
                (Grab::Move, Some(Item::Pin(i))) => {
                    let pin = &mut self.symbol.pins[i];
                    pin.at = pin.at.offset(delta);
                    pin.end = pin.end.offset(delta);
                }
                (Grab::Move, Some(Item::Graphic(i))) => match &mut self.symbol.graphics[i] {
                    Graphic::Path(points) => {
                        for p in points {
                            *p = p.offset(delta);
                        }
                    }
                    Graphic::Circle { center, .. } => *center = center.offset(delta),
                    Graphic::Text { at, .. } => *at = at.offset(delta),
                },
                // The end that was taken hold of follows the pointer, squared
                // to one of the four directions and to whole grid steps; the
                // other end does not move at all.
                (Grab::Tip | Grab::Root, Some(Item::Pin(i))) => {
                    let pin = &mut self.symbol.pins[i];
                    let anchor = if grab == Grab::Tip { pin.end } else { pin.at };
                    let reach = Point::new(q.x - anchor.x, q.y - anchor.y);
                    let length = grid_length(reach.x.abs().max(reach.y.abs()));
                    let moved = along(anchor, Aim::of(reach), length);
                    if grab == Grab::Tip {
                        pin.at = moved;
                    } else {
                        pin.end = moved;
                    }
                    self.pin_length = length;
                }
                _ => {}
            }
        }
        // Whether the gesture that just ended has already added something, so
        // a frame that reports BOTH a finished drag and a click does not add it
        // twice. The two tools that place on a bare click are the ones this
        // could double.
        let mut placed = false;
        if response.drag_stopped_by(PointerButton::Primary) {
            if let Some(drag) = self.drag.take() {
                self.record(drag.before);
            }
            if let Some(start) = self.start.take()
                && let Some(p) = pointer
            {
                self.add_item(start, world(p));
                placed = true;
            }
        }
        if response.clicked()
            && let Some(p) = pointer
        {
            if matches!(self.tool, DrawTool::Text | DrawTool::Pin) {
                if !placed {
                    self.add_item(world(p), world(p));
                }
            } else if self.tool == DrawTool::Select {
                self.selected = self.hit(p, screen);
            }
        }
        if response.secondary_clicked()
            && let Some(p) = pointer
        {
            self.selected = self.hit(p, screen);
        }
        response.context_menu(|ui| {
            menu_ui(ui);
            if self.selected.is_some() && ui.button("Delete item").clicked() {
                let before = self.symbol.clone();
                self.remove_selected();
                self.record(before);
                ui.close();
            }
        });
        painter.rect_filled(rect, 0., Color32::from_rgb(14, 22, 32));
        let grid = 1270. * zoom;
        if grid > 5. {
            let mut x = rect.left() + (center.x - rect.left()).rem_euclid(grid);
            while x < rect.right() {
                let mut y = rect.top() + (center.y - rect.top()).rem_euclid(grid);
                while y < rect.bottom() {
                    painter.circle_filled(Pos2::new(x, y), 1., Color32::from_gray(48));
                    y += grid;
                }
                x += grid;
            }
        }
        painter.line_segment(
            [center - Vec2::new(8., 0.), center + Vec2::new(8., 0.)],
            Stroke::new(1., Color32::GRAY),
        );
        painter.line_segment(
            [center - Vec2::new(0., 8.), center + Vec2::new(0., 8.)],
            Stroke::new(1., Color32::GRAY),
        );
        for (i, g) in self.symbol.graphics.iter().enumerate() {
            let color = if self.selected == Some(Item::Graphic(i)) {
                ACCENT
            } else {
                INK
            };
            match g {
                Graphic::Path(points) => {
                    painter.add(egui::Shape::line(
                        points.iter().map(|p| screen(*p)).collect(),
                        Stroke::new(2., color),
                    ));
                }
                Graphic::Circle { center, radius } => {
                    painter.circle_stroke(
                        screen(*center),
                        *radius as f32 * zoom,
                        Stroke::new(2., color),
                    );
                }
                Graphic::Text { at, text, size } => {
                    let galley = crate::zoomed_text(
                        ui.ctx(), text.clone(),
                        FontId::proportional((*size as f32 * zoom).max(2.)), color,
                    );
                    painter.galley(screen(*at), galley, color);
                }
            }
        }
        // A symbol split into gates names each over its box as a sheet will
        // (`U?A`), so which pins go with which gate is read off the canvas.
        // High enough to clear the number printed over a pin that leaves the
        // box's top edge, as an op-amp's power gate's V+ does: at 10 points
        // the caption ran into it.
        if self.symbol.gate_count() > 1 {
            for gate in 1..=self.symbol.gate_count() {
                if let Some((lo, hi)) = self.symbol.gate_extent(gate) {
                    painter.text(
                        screen(Point::new((lo.x + hi.x) / 2, lo.y)) - Vec2::new(0., 24.),
                        Align2::CENTER_BOTTOM,
                        format!("Gate {} · {}?{}", gate, self.symbol.reference_prefix, brep_ecad_core::gate_letter(gate)),
                        FontId::proportional(13.),
                        Color32::from_rgb(119, 137, 158),
                    );
                }
            }
        }
        for (i, p) in self.symbol.pins.iter().enumerate() {
            let selected = self.selected == Some(Item::Pin(i));
            let color = if selected { ACCENT } else { WIRE };
            // A pin that is the connection point of nothing is marked at its
            // TIP, which is the end that would have been one.
            let tip = if self.unbound(&p.number) {
                board_view::WARNING
            } else {
                color
            };
            painter.line_segment([screen(p.at), screen(p.end)], Stroke::new(2., color));
            painter.circle_stroke(screen(p.at), 4., Stroke::new(1.5, tip));
            // The selected pin wears its two grab handles: a disc on the TIP,
            // a square on the body END, each where a drag takes hold of it.
            if selected {
                painter.circle_filled(screen(p.at), 3.5, tip);
                painter.rect_filled(
                    Rect::from_center_size(screen(p.end), Vec2::splat(7.)),
                    1.,
                    color,
                );
            }
            painter.text(
                screen(p.at) + Vec2::new(3., -5.),
                Align2::LEFT_BOTTOM,
                &p.number,
                FontId::monospace(12.),
                color,
            );
            // Inside the body, past the body end and reading away from the tip, on
            // whichever side the pin leaves — the rule the schematic draws by.
            if let Some((anchor, align, name)) = pin_name_label(p, screen(p.at), screen(p.end)) {
                painter.text(anchor, align, name, FontId::monospace(12.), color);
            }
        }
        if let (Some(start), Some(p)) = (self.start, pointer) {
            let a = screen(start);
            let b = screen(world(p));
            let stroke = Stroke::new(1.5, ACCENT);
            match self.tool {
                DrawTool::Rectangle => {
                    painter.rect_stroke(
                        Rect::from_two_pos(a, b),
                        0.,
                        stroke,
                        egui::StrokeKind::Inside,
                    );
                }
                DrawTool::Circle => {
                    painter.circle_stroke(a, a.distance(b), stroke);
                }
                _ => {
                    painter.line_segment([a, b], stroke);
                }
            }
        }
    }
    fn add_item(&mut self, a: Point, b: Point) {
        if a == b && !matches!(self.tool, DrawTool::Text | DrawTool::Pin) {
            return;
        }
        let before = self.symbol.clone();
        let graphic = match self.tool {
            DrawTool::Line => Some(Graphic::Path(vec![a, b])),
            DrawTool::Rectangle => Some(Graphic::Path(vec![
                a,
                Point::new(b.x, a.y),
                b,
                Point::new(a.x, b.y),
                a,
            ])),
            DrawTool::Circle => Some(Graphic::Circle {
                center: a,
                radius: ((b.x - a.x) as f64).hypot((b.y - a.y) as f64).round() as i32,
            }),
            DrawTool::Text => Some(Graphic::Text {
                at: a,
                text: "Text".into(),
                size: 1270,
            }),
            DrawTool::Pin => {
                let number = self.free_number();
                // The drag runs from the electrical tip TOWARD the body, so the
                // pin leaves the body the other way; its length is how far the
                // drag reached. A bare click places one of the length the user
                // last worked in, leaving the body to the left — where the
                // first pin of a symbol drawn left to right goes.
                let reach = Point::new(b.x - a.x, b.y - a.y);
                let (aim, length) = if a == b {
                    (Aim::Left, self.pin_length)
                } else {
                    (
                        Aim::of(reach).opposite(),
                        grid_length(reach.x.abs().max(reach.y.abs())),
                    )
                };
                self.pin_length = length;
                self.selected = Some(Item::Pin(self.symbol.pins.len()));
                let gate = self.gate_at(a);
                self.symbol.pins.push(Pin {
                    hidden: false,
                    unit: 1,
                    gate,
                    number,
                    // Unnamed, as KiCad's `~`: its number already labels it, and a
                    // stock name would be drawn on every pin until each is renamed.
                    name: String::new(),
                    electrical_type: "passive".into(),
                    at: a,
                    end: along(a, aim.opposite(), length),
                });
                None
            }
            DrawTool::Select => None,
        };
        if let Some(g) = graphic {
            self.selected = Some(Item::Graphic(self.symbol.graphics.len()));
            // In a symbol split into gates, a new graphic is drawn with the gate
            // it was drawn over, so a sheet moves it with that gate.
            if !self.symbol.graphic_gates.is_empty() {
                let gate = self.gate_at(a);
                self.symbol.graphic_gates.resize(self.symbol.graphics.len(), 0);
                self.symbol.graphic_gates.push(gate);
            }
            self.symbol.graphics.push(g);
        }
        self.record(before);
    }
}
/// Where a pin's name is drawn, given its tip `at` and body end `end` on screen: the
/// anchor, how the text hangs from it, and the name — or `None` for an unnamed pin.
fn pin_name_label(pin: &Pin, at: Pos2, end: Pos2) -> Option<(Pos2, Align2, &str)> {
    let name = crate::pin_name(&pin.name)?;
    let (align, offset) = crate::pin_name_placement(at, end);
    Some((end + offset, align, name))
}
/// [`action`] for the symbol editor, so the table's closures need no type.
const fn symbol_action(
    id: &'static str,
    label: &'static str,
    group: ActionGroup,
    run: fn(&mut SymbolEditor),
) -> Action<SymbolEditor> {
    action(id, label, group, run)
}
/// Every command of the symbol editor, in toolbar order.
pub fn symbol_actions() -> &'static [Action<SymbolEditor>] {
    SYMBOL_ACTIONS
}
static SYMBOL_ACTIONS: &[Action<SymbolEditor>] = &[
    symbol_action("symbol.tool.select", "Select", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Select)
    })
    .keys(&[key(Key::Escape)])
    .pressed_when(|e| e.tool == DrawTool::Select),
    symbol_action("symbol.tool.line", "Line", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Line)
    })
    .pressed_when(|e| e.tool == DrawTool::Line),
    symbol_action(
        "symbol.tool.rectangle",
        "Rectangle",
        ActionGroup::Tool,
        |e| e.set_tool(DrawTool::Rectangle),
    )
    .pressed_when(|e| e.tool == DrawTool::Rectangle),
    symbol_action("symbol.tool.circle", "Circle", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Circle)
    })
    .pressed_when(|e| e.tool == DrawTool::Circle),
    symbol_action("symbol.tool.pin", "Pin", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Pin)
    })
    .pressed_when(|e| e.tool == DrawTool::Pin),
    symbol_action("symbol.tool.text", "Text", ActionGroup::Tool, |e| {
        e.set_tool(DrawTool::Text)
    })
    .pressed_when(|e| e.tool == DrawTool::Text),
    symbol_action(
        "symbol.undo",
        "Undo",
        ActionGroup::History,
        SymbolEditor::undo,
    )
    .keys(&[command(Key::Z)])
    .enabled_when(|e| e.can_step_history(HistoryKey::Undo)),
    symbol_action(
        "symbol.redo",
        "Redo",
        ActionGroup::History,
        SymbolEditor::redo,
    )
    .keys(&[
        egui::KeyboardShortcut::new(
            egui::Modifiers::COMMAND.plus(egui::Modifiers::SHIFT),
            Key::Z,
        ),
        command(Key::Y),
    ])
    .enabled_when(|e| e.can_step_history(HistoryKey::Redo)),
    symbol_action("symbol.fit", "Fit", ActionGroup::Zoom, |e| e.fit = true).keys(&[key(Key::F)]),
    symbol_action("symbol.zoom_in", "Zoom +", ActionGroup::Zoom, |e| {
        e.zoom = crate::scaled_zoom(e.zoom, 1.25)
    }),
    symbol_action("symbol.zoom_out", "Zoom −", ActionGroup::Zoom, |e| {
        e.zoom = crate::scaled_zoom(e.zoom, 1.0 / 1.25)
    }),
    symbol_action(
        "symbol.delete",
        "Delete item",
        ActionGroup::Selection,
        |e| {
            let before = e.symbol.clone();
            e.remove_selected();
            e.record(before);
        },
    )
    .keys(&[key(Key::Delete), key(Key::Backspace)])
    .enabled_when(|e| e.selected.is_some()),
    symbol_action(
        "symbol.pin.rotate",
        "Rotate pin 90°",
        ActionGroup::Selection,
        SymbolEditor::rotate_pin,
    )
    .keys(&[key(Key::R)])
    .enabled_when(SymbolEditor::pin_selected),
    // Length without a pointer. Two stepped commands rather than one taking a
    // value: every action in these tables takes none, a host runs them by id
    // alone, and a schematic pin's length moves in whole grid steps anyway —
    // the exact figure is the `Length` field's.
    symbol_action(
        "symbol.pin.lengthen",
        "Lengthen pin",
        ActionGroup::Selection,
        |e| e.step_pin_length(1),
    )
    .keys(&[key(Key::CloseBracket)])
    .enabled_when(SymbolEditor::pin_selected),
    symbol_action(
        "symbol.pin.shorten",
        "Shorten pin",
        ActionGroup::Selection,
        |e| e.step_pin_length(-1),
    )
    .keys(&[key(Key::OpenBracket)])
    .enabled_when(SymbolEditor::pin_can_shorten),
    // A pin that binds nothing is found without scrolling a long list by eye:
    // the Items list names them, and this selects the next after the one in
    // hand, which the list then scrolls to.
    symbol_action(
        "symbol.pin.next_unbound",
        "Next unbound pin",
        ActionGroup::Selection,
        SymbolEditor::select_next_unbound,
    )
    .keys(&[key(Key::U)])
    .enabled_when(|e| e.unbound_pins().next().is_some()),
];

/// How near the pointer must come to one of a pin's ends to take hold of it:
/// [`HANDLE`], but never more than a third of the pin as it is drawn, so the
/// two handles of a very short pin leave the body between them free to be
/// dragged — which is what moves the whole pin.
fn handle_reach(tip: Pos2, root: Pos2) -> f32 {
    HANDLE.min(tip.distance(root) / 3.)
}

/// The Items list's height when nothing else says: the nine rows it always
/// had, which is also its height in a short pane.
const LIST_HEIGHT: f32 = 190.;
/// The share of a tall pane the Items list takes, so a 40-pin symbol shows
/// more of its rows where there is room for them.
pub(crate) const LIST_SHARE: f32 = 0.3;
/// The shortest the list's grip will drag it: three rows.
const LIST_MIN: f32 = 64.;

/// A list's height: the one the user dragged it to, else `share` of the pane
/// (read before the Inspector's scroll area made it endless), never less than
/// `floor`, the height it had before it could be sized.
pub(crate) fn list_height(set: Option<f32>, pane: f32, share: f32, floor: f32) -> f32 {
    set.unwrap_or(if pane.is_finite() { (pane * share).max(floor) } else { floor })
}

/// The grip under `list`, drawn when the list scrolls or the user has sized it
/// (a short list shows every row and has none), dragging `height` between
/// three rows and the whole list; a double click forgets it. The grip, when
/// drawn, for the Inspector to publish.
pub(crate) fn grip_under<R>(
    ui: &mut egui::Ui,
    list: &egui::scroll_area::ScrollAreaOutput<R>,
    height: &mut Option<f32>,
) -> Option<egui::Response> {
    let scrolls = list.content_size.y > list.inner_rect.height() + 0.5;
    if !scrolls && height.is_none() {
        return None;
    }
    let grip = list_grip(ui);
    if grip.dragged() {
        let dragged = list.inner_rect.height() + grip.drag_delta().y;
        *height = Some(dragged.clamp(LIST_MIN, list.content_size.y.max(LIST_MIN)));
    }
    if grip.double_clicked() {
        *height = None;
    }
    Some(grip)
}

/// The grip under a list that scrolls: drag it down to show more rows, up to
/// show fewer; a double click goes back to the height the pane gives.
pub(crate) fn list_grip(ui: &mut egui::Ui) -> egui::Response {
    let (rect, response) =
        ui.allocate_exact_size(Vec2::new(ui.available_width(), 10.), Sense::click_and_drag());
    let visuals = ui.visuals();
    let colour = if response.hovered() || response.dragged() {
        visuals.strong_text_color()
    } else {
        visuals.weak_text_color()
    };
    let c = rect.center();
    for dy in [-1.5, 1.5] {
        ui.painter().line_segment(
            [Pos2::new(c.x - 16., c.y + dy), Pos2::new(c.x + 16., c.y + dy)],
            Stroke::new(1., colour),
        );
    }
    response
        .on_hover_cursor(egui::CursorIcon::ResizeVertical)
        .on_hover_text("Drag to show more rows or fewer. Double-click to go back.")
}

/// The Inspector's clickable widgets, `inspector:<thing>`, where the editor's
/// `properties` last drew them — the second half of its `hits`, which the host
/// publishes beside the canvas's. A widget is published only while at least
/// half of it is in view, clipped to what is: one scrolled out of the
/// Inspector, or out of a list in it, is not there to be clicked. And only
/// while the Inspector is being drawn at all: a pane behind another tab keeps
/// no rects ([`Self::shown`]).
#[derive(Default)]
pub(crate) struct InspectorHits {
    pub(crate) rects: Vec<(String, Rect)>,
    /// The pass the Inspector was last drawn in.
    drawn: Option<u64>,
}
impl InspectorHits {
    /// Start a frame's rects afresh.
    pub(crate) fn begin(&mut self, ctx: &egui::Context) {
        self.rects.clear();
        self.drawn = Some(ctx.cumulative_pass_nr());
    }
    /// `inspector:<key>` is drawn where `response` is, as far as it is shown.
    pub(crate) fn mark(&mut self, key: impl std::fmt::Display, response: &egui::Response) {
        let (rect, seen) = (response.rect, response.interact_rect);
        if seen.is_positive() && seen.height() * 2. >= rect.height() {
            self.rects.push((format!("inspector:{key}"), seen));
        }
    }
    /// The rects, if the Inspector was drawn in the canvas's pass `canvas` or
    /// the one before: the two panes are drawn in turn, and either may come
    /// first. Nothing once a pass has gone by without it.
    pub(crate) fn shown(&self, canvas: u64) -> Vec<(String, Rect)> {
        match self.drawn {
            Some(drawn) if drawn + 1 >= canvas => self.rects.clone(),
            _ => vec![],
        }
    }
}

/// A widget's name as a key word: `Body end` is `body_end`.
pub(crate) fn key_word(name: &str) -> String {
    name.trim().to_lowercase().replace(' ', "_")
}

/// What a pin's connection point says when hovered. A user who has never seen
/// the ports model needs to be told once that the address is not a second
/// thing to keep in step with the pin.
const POINT_HINT: &str = "This pin IS this connection point: one name, on the symbol, on the pads and in 3D. Rename the pin and the point follows.";

/// What the orientation buttons say when hovered: the one place the three ways
/// of turning a pin are named together.
const TURN_HINT: &str = "Which way the pin leaves the body. Drag either end of the selected pin on the canvas, or press R to turn it and [ or ] to shorten or lengthen it.";

/// A point's two fields, keyed `<name>:x` and `<name>:y` ([`key_word`]).
fn point_fields(ui: &mut egui::Ui, hits: &mut InspectorHits, name: &str, p: &mut Point) {
    ui.push_id(name, |ui| {
        ui.label(name);
        let key = key_word(name);
        mm_field(ui, hits, &format!("{key}:x"), &mut p.x, -100000. ..=100000.);
        mm_field(ui, hits, &format!("{key}:y"), &mut p.y, -100000. ..=100000.);
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
                .speed(0.127)
                .range(range)
                .suffix(" mm"),
        );
        hits.mark(key, &field);
        if field.changed() {
            *value = (mm * 1000.).round() as i32;
        }
    });
}


/// The hit key of the pin at `index` of its symbol: `pin:<number>`, or, for a
/// pin whose number is blank, `unnumbered-pin:<index>`. A pin's number may be
/// any other text, so every key under `pin:` could be some pin's number, and
/// `pin:` bare is the family's own prefix: a blank number published there
/// was returned by every `pin:` query, and two of them were one key. Out of
/// the family, by index — the one thing a numberless pin still has.
fn pin_key(index: usize, pin: &Pin) -> String {
    if pin.number.trim().is_empty() {
        format!("unnumbered-pin:{index}")
    } else {
        format!("pin:{}", pin.number)
    }
}

/// Where the symbol point `p` is drawn on a canvas centred (with its pan) at
/// `center` — the one mapping the canvas paints with and [`SymbolEditor::hits`]
/// reports with.
fn on_canvas(center: Pos2, zoom: f32, p: Point) -> Pos2 {
    center + Vec2::new(p.x as f32, p.y as f32) * zoom
}
