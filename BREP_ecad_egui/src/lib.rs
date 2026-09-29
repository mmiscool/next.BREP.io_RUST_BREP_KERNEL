//! An embeddable schematic and PCB editor. The host owns file I/O and application chrome.
use std::collections::BTreeMap;
use brep_ecad_core::{
    Component, ConnectionTarget, Document, DocumentKind, Graphic, History, Label, PartSource,
    Point, Symbol, Terminal, Uuid,
};
use egui::{Align2, Color32, FontId, Key, PointerButton, Pos2, Rect, Sense, Stroke, Vec2};

mod actions;
mod board_view;
mod erc_panel;
mod footprint_editor;
mod symbol_editor;
pub use actions::{Action, ActionGroup, actions};
pub use board_view::{BoardSelection, BoardTool, View};
pub use footprint_editor::{FootprintEditor, pad_actions};
pub use symbol_editor::{PinPoints, SymbolEditor, symbol_actions};

/// Keep zoom free of product limits, rejecting only scales that cannot be
/// represented by the canvas math (including projected integer coordinates).
fn valid_zoom(candidate: f32, previous: f32) -> f32 {
    if candidate > 0.0
        && candidate.recip().is_finite()
        && (candidate * i32::MAX as f32).is_finite()
    {
        candidate
    } else {
        previous
    }
}

fn scaled_zoom(current: f32, factor: f32) -> f32 {
    valid_zoom(current * factor, current)
}

/// Preserve the world point under an anchor while scaling. Commit both pieces
/// together, so a numeric overflow cannot leave a broken pan behind.
fn zoom_at(zoom: &mut f32, pan: &mut Vec2, factor: f32, anchor: Vec2) {
    let next = scaled_zoom(*zoom, factor);
    let next_pan = anchor - (anchor - *pan) * (next / *zoom);
    if next_pan.x.is_finite() && next_pan.y.is_finite() {
        *zoom = next;
        *pan = next_pan;
    }
}

/// Coarsen the displayed grid by whole multiples of its snapping grid. Iteration
/// is in screen space, so zooming out cannot enumerate millions of world points.
fn grid_spacing(zoom: f32) -> f32 {
    let mut spacing = 1270.0 * zoom;
    while spacing > 0.0 && spacing < 8.0 {
        spacing *= 5.0;
    }
    spacing
}

/// Large zoomed labels must scale their drawing, not allocate enormous glyphs
/// in the font atlas. Keep ordinary sizes unchanged and scale the cached mesh
/// above that size; this is a rasterization budget, not a view zoom limit.
fn zoomed_text(ctx: &egui::Context, text: String, mut font: FontId, color: Color32) -> std::sync::Arc<egui::Galley> {
    let scale = (font.size / 96.0).max(1.0);
    font.size /= scale;
    let galley = ctx.fonts_mut(|fonts| fonts.layout_no_wrap(text, font, color));
    if scale == 1.0 {
        return galley;
    }
    let mut shape = egui::epaint::TextShape::new(Pos2::ZERO, galley, color);
    shape.transform(egui::emath::TSTransform::from_scaling(scale));
    shape.galley
}

const INK: Color32 = Color32::from_rgb(211, 220, 233);
const ACCENT: Color32 = Color32::from_rgb(89, 218, 180);
const WIRE: Color32 = Color32::from_rgb(93, 186, 228);
/// Emphasis in panel text: the theme's own accent, so a label reads on the light
/// theme as well as the dark. [`ACCENT`] is for the canvases, which paint their own
/// dark ground; on a light panel it measured 1.63:1 against a 4.5:1 floor.
pub(crate) fn accent(ui: &egui::Ui) -> Color32 {
    ui.visuals().selection.stroke.color
}
/// Whether an eCAD editor that is up takes this frame's keys: every key, wherever
/// the pointer is, unless a TEXT FIELD has the keyboard (in the Inspector, a side
/// pane, the tool card or a window). All four editors ask this before their
/// shortcuts: Diagram and PCB in their own `show`, Symbol and Pads in their
/// `run_shortcuts`. The host's Delete and Escape on an eCAD workbench ask it too. Not
/// `egui_wants_keyboard_input`: that is ANY focused widget, and a button reached by
/// Tab would take R from the editor while doing nothing with it.
///
/// Nor while a MENU or a combo box is open: it owns the keys, and it closes on
/// `key_pressed(Escape)`, which a key the editor consumed never is. The canvas's own
/// right-click menu is drawn after the editor's keys run, so without this Escape
/// cancelled the editor's selection and left the menu standing over it. Tooltips
/// are not counted: they are not held open in egui's memory.
///
/// Nor under a MODAL (`egui::Modal`, a host's file dialog or eCAD's own
/// autoroute dialog): it owns every key, and its Escape closes it. egui keeps
/// the top modal layer of the LAST pass, which is what this reads, so it covers
/// an editor drawn by any host and needs no list of the modals. On the one
/// frame a modal first opens it reads nothing: egui's current-pass record is
/// private (`Focus::top_modal_layer_current_frame`), and a modal drawn after
/// the editor has not been drawn when the editor's keys run in any case.
pub fn keys_reach_editor(ctx: &egui::Context) -> bool {
    !ctx.text_edit_focused()
        && !egui::Popup::is_any_open(ctx)
        && ctx.memory(|memory| memory.top_modal_layer()).is_none()
}
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tool {
    Select,
    Wire,
    Junction,
    Label,
    Place,
    Move,
    /// Marks a pin as meant to be left open, KiCad's no-connect ✕; a second click
    /// takes the mark off.
    NoConnect,
    /// Puts KiCad's PWR_FLAG on a pin, or on the net under the click: the net is
    /// driven from off the sheet. A second click takes it off.
    PowerFlag,
    /// Places a power symbol for [`Editor::power_net_text`]: a part with one power
    /// input pin whose net takes the symbol's name, so every symbol of one name is
    /// one net, as in KiCad.
    Power,
}
/// The power symbols the Power tool offers by name; any other name may be typed.
pub const POWER_NETS: [&str; 4] = ["GND", "VCC", "+3V3", "+5V"];
/// Whether a power net's symbol is drawn as a ground, bars below its pin, rather
/// than as a supply, an arrow above it.
pub fn is_ground(name: &str) -> bool {
    let name = name.to_ascii_uppercase();
    name.contains("GND") || name.starts_with("VSS") || name.starts_with("VEE") || name.starts_with('-')
}
/// The power symbol for net `name`, as KiCad's `power` library draws one: a single
/// hidden power input pin at its origin, named for the net, under reference
/// `#PWR`. [`Document::place`] names the pin's net after [`Symbol::power_net`], so
/// every power symbol of one name is one net; the board and the parts list leave
/// it out, as they leave out KiCad's. Its pin is a power INPUT, as KiCad's is, so
/// it does not drive the net: a net fed from off the sheet still wants a power
/// flag ([`Tool::PowerFlag`]).
pub fn power_symbol(name: &str) -> Symbol {
    let p = Point::new;
    let graphics = if is_ground(name) {
        // KiCad's GND: a stem down from the pin and a triangle under it.
        vec![Graphic::Path(vec![
            p(0, 0),
            p(0, 1270),
            p(1270, 1270),
            p(0, 2540),
            p(-1270, 1270),
            p(0, 1270),
        ])]
    } else {
        // KiCad's +5V: a stem up from the pin and an arrowhead on it.
        vec![
            Graphic::Path(vec![p(0, 0), p(0, -2540)]),
            Graphic::Path(vec![p(-762, -1270), p(0, -2540), p(762, -1270)]),
        ]
    };
    Symbol {
        unit_count: 1,
        properties: BTreeMap::from([("Value".to_owned(), name.to_owned())]),
        power_net: Some(name.to_owned()),
        library_id: format!("power:{name}"),
        reference_prefix: "#PWR".into(),
        description: format!("Power symbol: the {name} net"),
        graphics,
        pins: vec![brep_ecad_core::Pin {
            hidden: true,
            unit: 0,
            gate: 0,
            number: "1".into(),
            name: name.to_owned(),
            electrical_type: "power_in".into(),
            at: Point::default(),
            end: Point::default(),
        }],
        graphic_gates: vec![],
        hide_pin_names: true,
        hide_pin_numbers: true,
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Selection {
    Component(Uuid),
    Wire(Uuid),
    Label(Uuid),
    Junction(Point),
}

/// Private payload type prevents collisions with the embedding host's drag data.
struct LibraryPartDrag(Part);

/// A part the host offers to place: its own key and version, the name to show, the
/// symbol and pads a placed component copies, and the label it goes by.
#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    pub source: PartSource,
    pub name: String,
    pub symbol: Symbol,
    pub pads: Option<brep_ecad_core::board::Footprint>,
    /// The label the placed component takes, such as `J1`, when the host keeps the
    /// labels; `None` takes the next free one for the symbol's prefix. A part whose
    /// label is already on the sheet shows as placed and is not placed again.
    pub reference: Option<String>,
}

/// Opening setup for a context menu or a dropdown list. egui lays these out
/// justified, so an item is only as wide as the space the pointer happened to leave
/// and its label wraps in the gap. Keep labels on one line and give the list a width
/// to start from; it still grows to whatever the longest item needs.
fn menu_ui(ui: &mut egui::Ui) {
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
    ui.set_min_width(170.);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum HistoryKey {
    Undo,
    Redo,
}
/// An edit the host has not taken yet; see [`Editor::take_change`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Change {
    /// Set for an edit repeated as the user types in one field of one object, such as a
    /// component's reference. Consecutive changes with the same key may be kept as one
    /// undo step.
    pub coalesce: Option<String>,
}

impl Change {
    /// Merge edits until the host takes them. A mixed batch cannot coalesce
    /// with the host's previous undo step, even if its last key matches.
    fn record(pending: &mut Option<Self>, coalesce: Option<String>) {
        let coalesce = match pending.take() {
            Some(change) if change.coalesce != coalesce => None,
            _ => coalesce,
        };
        *pending = Some(Self { coalesce });
    }
}

struct NetFlagEdit {
    terminal: Terminal,
    name: String,
    focus: bool,
}

pub struct Editor {
    pub document: Document,
    pub history: History,
    /// The parts the host offers to place, shown in the library panel. Components
    /// are placed from parts only: a part carries its symbol and pads.
    pub parts: Vec<Part>,
    /// Shown in the library panel while the host offers no parts: where they come from.
    pub parts_hint: String,
    /// The keys of the parts the user asked to open, newest last, for the host to take
    /// and open where it edits a part's symbol and pads.
    pub open_part_requests: Vec<String>,
    /// Set when the host can add a part to what [`Editor::parts`] lists: the label of
    /// the library panel's button that asks it to. `None` draws no button, since a
    /// button the host cannot answer would be a dead end of its own.
    pub add_part_label: Option<String>,
    /// Set by that button, for the host to take and answer.
    pub add_part_requested: bool,
    /// The library panel's widgets as last drawn, keyed for automation (`parts:add`,
    /// `parts:search`, `parts:card:<reference, else name>`), for the host to take each frame. Empty
    /// when the panel was not drawn, so a hidden panel publishes nothing stale.
    pub library_hits: Vec<(String, Rect)>,
    /// Set when the host keeps the labels of placed parts ([`Part::reference`]): a
    /// component placed from a part then shows its reference instead of editing it,
    /// with this text saying where it is set.
    pub labels_from_host: Option<String>,
    placing_part: Option<Part>,
    pub tool: Tool,
    pub selected: Option<Selection>,
    /// Which gate of the selected component the pointer picked, for a
    /// component whose gates are placed apart; see [`Editor::selected_gate`].
    selected_gate: Option<(Uuid, u32)>,
    pub label_text: String,
    /// The net the Power tool's next symbol is for, such as `GND`.
    pub power_net_text: String,
    /// Set when the Label tool places a label, for the host's name field to take the
    /// keyboard back with its text selected: see [`Editor::take_label_placed`].
    label_placed: bool,
    /// Stock part number given to new connections in a wiring diagram.
    pub stock_part_number: String,
    pub status: String,
    /// Why the last edit on the sheet was refused, or what it could not do; shown until
    /// the next press there.
    notice: Option<String>,
    /// What the last edit on the sheet did that is worth a line and no alarm,
    /// such as a label joining a pin to the net its name already names: shown in
    /// the text colour, until the next press there. A [`Editor::notice`] wins.
    note: Option<String>,
    /// [`Editor::offered_mark`] at the last automatic board pass, so the pass runs when
    /// the host's offer of devices MOVES and not on the frames between. Deliberately
    /// NOT cleared by [`Editor::set_document`]: the host pulls its own copy of the
    /// document back under the editor on every undo, and a pass that ran again there
    /// would put back exactly what the undo took away.
    followed_parts: Option<u64>,
    /// [`Editor::stale_mark`] of the board state the last automatic pass in
    /// [`Editor::show`] healed FROM, so the same state arriving again — which is what an
    /// app-level undo looks like from here — is left alone. Deliberately NOT cleared by
    /// [`Editor::set_document`], for the same reason as `followed_parts`.
    healed_stale: Option<u64>,
    /// The components already named as wanting a place on the board and having no pads
    /// to take one with; see [`Editor::announce_without_pads`].
    reported_without_pads: Vec<String>,
    /// Edits since the host last called [`Editor::take_change`].
    pending_change: Option<Change>,
    pub changed: bool,
    search: String,
    placing: Option<Symbol>,
    rotation: u8,
    wire_start: Option<Point>,
    vertical_first: bool,
    pan: Vec2,
    zoom: f32,
    drag: Option<(Document, Point, Point)>,
    canvas_size: Vec2,
    terminal_drag: Option<ConnectionTarget>,
    path_drag: Option<(Document, Uuid, usize, Point)>,
    selected_segment: usize,
    context_terminal: Option<Terminal>,
    net_flag_edit: Option<NetFlagEdit>,
    pub view: View,
    board_view: board_view::BoardView,
    /// The Connectivity panel's electrical rule check ([`erc_panel`]).
    erc: erc_panel::ErcView,
    /// The pass the sheet was last drawn in, for the Inspector's rects.
    canvas_pass: u64,
    /// A part the palette just put down: the Inspector's Value field takes the
    /// keyboard once, with the symbol's value selected, so `10k` is typed over it
    /// as the part lands (the third eCAD audit, B3).
    value_prompt: Option<Uuid>,
    /// The Properties section's rects (`inspector:value`, `inspector:value_siblings`).
    properties_hits: symbol_editor::InspectorHits,
}
impl Default for Editor {
    fn default() -> Self {
        Self {
            document: Document::default(),
            history: History::default(),
            parts: vec![],
            parts_hint: "No parts to place.".into(),
            open_part_requests: vec![],
            add_part_label: None,
            add_part_requested: false,
            library_hits: vec![],
            labels_from_host: None,
            placing_part: None,
            tool: Tool::Select,
            selected: None,
            selected_gate: None,
            label_text: "SIGNAL".into(),
            power_net_text: "GND".into(),
            label_placed: false,
            stock_part_number: String::new(),
            status: "Choose a part to begin".into(),
            notice: None,
            note: None,
            followed_parts: None,
            healed_stale: None,
            reported_without_pads: vec![],
            pending_change: None,
            changed: false,
            search: String::new(),
            placing: None,
            rotation: 0,
            wire_start: None,
            vertical_first: false,
            pan: Vec2::ZERO,
            zoom: 0.012,
            drag: None,
            canvas_size: Vec2::new(800., 600.),
            terminal_drag: None,
            path_drag: None,
            selected_segment: 0,
            context_terminal: None,
            net_flag_edit: None,
            view: View::Schematic,
            board_view: Default::default(),
            erc: Default::default(),
            canvas_pass: 0,
            value_prompt: None,
            properties_hits: Default::default(),
        }
    }
}
impl Editor {
    pub fn replace_document(&mut self, document: Document) {
        self.document = document;
        self.history = History::default();
        self.selected = None;
        self.drag = None;
        self.terminal_drag = None;
        self.path_drag = None;
        self.net_flag_edit = None;
        self.context_terminal = None;
        self.wire_start = None;
        self.notice = None;
        self.note = None;
        self.followed_parts = None;
        self.healed_stale = None;
        self.reported_without_pads = vec![];
        self.pending_change = None;
        self.tool = Tool::Select;
        self.changed = false;
        self.pan = Vec2::ZERO;
        self.view = View::Schematic;
        self.board_view = Default::default();
        self.erc = Default::default();
    }
    /// Load a document the host keeps its own copy of, such as after the host's undo or
    /// on returning to its tab. Unlike [`Editor::replace_document`] this keeps pan, zoom,
    /// and the view. Selection, gestures in
    /// progress, and the editor's own undo history are cleared, and the load is not
    /// reported by [`Editor::take_change`].
    pub fn set_document(&mut self, document: Document) {
        self.cancel();
        self.reset_board_gestures();
        self.document = document;
        self.history = History::default();
        self.selected = None;
        self.net_flag_edit = None;
        self.context_terminal = None;
        self.notice = None;
        self.note = None;
        self.pending_change = None;
        self.changed = false;
        if self.wiring() && self.view == View::Board {
            self.view = View::Schematic;
        }
    }
    /// Bring the sheet's multi-unit parts saved before gates existed up to the gate
    /// shape ([`Document::migrate_legacy_units`]), and report that to the host as a
    /// change, OUTSIDE this editor's own undo. Returns the references it changed.
    ///
    /// A host calls it on the frame it first loads the stored sheet, and writes what
    /// is reported then as part of the document with no undo step, as it does the
    /// board a sheet saved before the board followed it gets on that frame. It is not
    /// run on every load: the host's undo also loads the sheet, and a migration there
    /// would be a fresh edit that ends the user's redo.
    pub fn adopt_legacy_units(&mut self) -> Vec<String> {
        let migrated = self.document.migrate_legacy_units();
        if !migrated.is_empty() {
            self.changed = true;
            self.note_change(None);
        }
        migrated
    }
    /// The edits made since the last call, merged into one, or `None` when the document
    /// has not changed. A host that saves the document inside its own file calls this
    /// once a frame and stores [`Editor::document`] when it returns a change. Undo and
    /// redo in the editor count as changes.
    pub fn take_change(&mut self) -> Option<Change> {
        self.pending_change.take()
    }
    /// Where the view in front is looking — the sheet's or, on the board view, the
    /// board's: the drawing origin's offset from the canvas centre in points, and the
    /// zoom in points per micrometre. Read-only, for a host that shows the view.
    pub fn pan_zoom(&self) -> (Vec2, f32) {
        match self.view {
            View::Board => self.board_view.pan_zoom(),
            View::Schematic => (self.pan, self.zoom),
        }
    }
    fn note_change(&mut self, coalesce: Option<String>) {
        Change::record(&mut self.pending_change, coalesce);
    }
    /// Record a finished edit that started from `before`, for undo and for the host.
    fn commit(&mut self, before: Document, coalesce: Option<String>) {
        // The board follows the sheet INSIDE the edit that moved it: recorded below as
        // one undo step and reported to the host as one change, with the edit's own
        // coalesce key, so a part placed on the schematic lands on the board without a
        // view switch and without a second step to undo. The gate is two id sets
        // ([`Editor::board_is_stale`]); the sweep behind it runs on the edits that add
        // or remove a component and on no others.
        if self.board_is_stale() {
            let sync = self.document.sync_board();
            self.announce(&sync, &[]);
        }
        if before != self.document {
            self.changed = true;
            self.board_view.violations = None;
            self.note_change(coalesce);
        }
        self.history.record(before, &self.document);
    }
    /// Apply one undoable edit. Returns false when the edit was refused because it would
    /// break a wiring diagram's rules; the document is then unchanged.
    fn transaction(&mut self, f: impl FnOnce(&mut Document)) -> bool {
        self.transaction_as(None, f)
    }
    /// A transaction whose repeats, such as typing in one field, share `coalesce`.
    fn transaction_as(&mut self, coalesce: Option<String>, f: impl FnOnce(&mut Document)) -> bool {
        let before = self.document.clone();
        f(&mut self.document);
        // Core edits already keep a wiring diagram valid; this catches any that would not,
        // so the document can always be saved. Drags that bypass this only move devices,
        // slide segments between fixed ends, or move labels, which a wiring diagram lacks.
        if self.document.kind == DocumentKind::Wiring
            && before != self.document
            && before.validate().is_ok()
            && let Err(e) = self.document.validate()
        {
            self.document = before;
            self.notice = Some(e);
            return false;
        }
        self.commit(before, coalesce);
        true
    }
    fn wiring(&self) -> bool {
        self.document.kind == DocumentKind::Wiring
    }
    /// The occurrence a component is bound to and the host does not offer — UNLINKED.
    ///
    /// A component names its device by OCCURRENCE ([`PartSource::instance`]), which
    /// changes whenever the device is moved in the assembly tree, and the host's
    /// `parts` are the devices that assembly reaches. So a bound occurrence the offer
    /// does not carry is a broken link, and the answer is the user's:
    /// [`Editor::relink_picker`] asks which device it is and nothing here guesses.
    ///
    /// Read where the host owns the parts ([`Editor::labels_from_host`]) and nowhere
    /// else: on a sheet whose components come from a library rather than from an
    /// assembly, `parts` says nothing about which occurrences exist.
    ///
    /// This is a SUPERSET of the host's own UNLINKED test (`ecad_parts::follow_sheet`),
    /// by one case: a device the assembly reaches whose part its library does not carry
    /// is reached there and cannot be offered here, so it reads as unlinked. The host
    /// reports that state too, in its own words, and picking another device is a fair
    /// answer to it.
    fn unlinked_device<'a>(&self, c: &'a Component) -> Option<&'a str> {
        if self.labels_from_host.is_none() {
            return None;
        }
        let lost = c.part.as_ref()?.instance.as_deref()?;
        let offered = self
            .parts
            .iter()
            .any(|p| p.source.instance.as_deref() == Some(lost));
        (!offered).then_some(lost)
    }
    /// [`Editor::unlinked_device`] over the whole sheet, in sheet order.
    fn unlinked(&self) -> Vec<(Uuid, &str)> {
        self.document
            .components
            .iter()
            .filter_map(|c| Some((c.id, self.unlinked_device(c)?)))
            .collect()
    }
    /// The devices an unlinked component could be bound to, each with the pins it would
    /// cost, in the order they are offered in.
    ///
    /// Every device the host offers that no component of this sheet already carries —
    /// [`Document::relink`] refuses one that is placed twice, as placing it twice is
    /// refused. A device of the same PART the component was placed from comes first,
    /// because a moved or re-versioned occurrence is the likely answer. That is an
    /// ORDER and not a guess: nothing is preselected, one is never applied on its own,
    /// and a single candidate is offered exactly as five are.
    fn relink_candidates(&self, id: Uuid) -> Vec<(Part, Vec<String>)> {
        let taken: std::collections::BTreeSet<&str> = self
            .document
            .components
            .iter()
            .filter(|c| c.id != id)
            .filter_map(|c| c.part.as_ref()?.instance.as_deref())
            .collect();
        let own = self
            .document
            .components
            .iter()
            .find(|c| c.id == id)
            .and_then(|c| c.part.as_ref())
            .map(|p| p.key.as_str());
        let mut candidates: Vec<(Part, Vec<String>)> = self
            .parts
            .iter()
            .filter(|p| {
                p.source
                    .instance
                    .as_deref()
                    .is_some_and(|instance| !taken.contains(instance))
            })
            .map(|p| (p.clone(), self.dropped_pins(id, &p.symbol)))
            .collect();
        // Stable, so the assembly's own order — a parent before its children — survives
        // inside each half.
        candidates.sort_by_key(|(p, _)| own.is_some_and(|key| key != p.source.key));
        candidates
    }
    /// The pins of a component that carry a wire or a net label and that `symbol` has
    /// not got, so binding the component to it would drop them.
    ///
    /// [`Document::relink`] takes a rename map and this editor has none to give: a
    /// rename is evidence two saved versions of ONE part carry, and a different device
    /// is not that. So a pin is kept when the new symbol has one by the same name and
    /// dropped when it has not, and the cost is shown on the candidate before it is
    /// picked rather than discovered after.
    fn dropped_pins(&self, id: Uuid, symbol: &Symbol) -> Vec<String> {
        let kept: std::collections::BTreeSet<&str> =
            symbol.pins.iter().map(|p| p.number.as_str()).collect();
        let mut pins: Vec<String> = self
            .document
            .wires
            .iter()
            .flat_map(|w| [w.start.as_ref(), w.end.as_ref()])
            .chain(self.document.labels.iter().map(|l| l.terminal.as_ref()))
            .flatten()
            .filter(|t| t.component == id && !kept.contains(t.pin.as_str()))
            .map(|t| t.pin.clone())
            .collect();
        pins.sort();
        pins.dedup();
        pins
    }
    /// Offer the devices an UNLINKED component could be bound to, and bind it to the one
    /// the user picks. The resolving half of the UNLINKED lane; the operation is
    /// [`Document::relink`] and the decision is the user's.
    fn relink_picker(&mut self, ui: &mut egui::Ui, id: Uuid, lost: &str) {
        ui.add_space(6.);
        ui.label(
            egui::RichText::new("UNLINKED")
                .color(board_view::WARNING)
                .strong(),
        );
        ui.small(format!(
            "This component is bound to the occurrence '{lost}', which this assembly no \
             longer offers. Pick the device it belongs to."
        ));
        let candidates = self.relink_candidates(id);
        if candidates.is_empty() {
            ui.small("No device of this assembly is free to take it.");
            return;
        }
        let mut chosen = None;
        egui::ScrollArea::vertical()
            .id_salt(("relink", id))
            .max_height(170.)
            .show(ui, |ui| {
                for (part, dropped) in &candidates {
                    let label = part.reference.as_deref().unwrap_or(&part.name);
                    let caption = match dropped.len() {
                        0 => format!("{label}  ·  {}", part.source.key),
                        n => format!("{label}  ·  {}  ·  drops {n}", part.source.key),
                    };
                    let hover = match dropped.is_empty() {
                        true => format!(
                            "Takes {}'s symbol and pads. Where this component sits, what it \
                             is called and every wire on it stay as they are.",
                            part.source.key
                        ),
                        false => format!(
                            "{} has no {} {}, so the connections there are dropped. \
                             Everything else stays as it is.",
                            part.source.key,
                            match dropped.len() {
                                1 => "pin",
                                _ => "pins",
                            },
                            dropped.join(", ")
                        ),
                    };
                    if ui.button(caption).on_hover_text(hover).clicked() {
                        chosen = Some(part.clone());
                    }
                }
            });
        if let Some(part) = chosen {
            let mut done = Ok(());
            self.transaction(|d| {
                done = d.relink(
                    id,
                    part.source.clone(),
                    &part.symbol,
                    part.pads.as_ref(),
                    &Default::default(),
                );
            });
            match done {
                Ok(()) => {
                    self.notice = None;
                    self.note = None;
                    self.board_view.message = format!(
                        "Linked to {}.",
                        part.reference.unwrap_or(part.source.key)
                    );
                }
                Err(refusal) => self.notice = Some(refusal),
            }
        }
    }
    /// Start placing one of the host's parts: the next click on the sheet puts it down
    /// with a copy of its symbol and pads.
    pub fn begin_placing_part(&mut self, part: Part) {
        self.placing = Some(part.symbol.clone());
        self.placing_part = Some(part);
        self.tool = Tool::Place;
        self.rotation = 0;
    }
    /// Place the part being placed, or `symbol` for a copy of a component drawn
    /// without a part. A part that carries its label is placed once: the tool is put
    /// down after it, and a label already on the sheet is refused with a notice.
    fn place_here(&mut self, symbol: Symbol, at: Point, rotation: u8) -> Option<Uuid> {
        let part = self.placing_part.clone();
        let labelled = part.as_ref().is_some_and(|p| p.reference.is_some());
        let mut placed = Ok(None);
        self.transaction(|d| {
            placed = match part {
                Some(part) => d
                    .place_part(part.source, part.symbol, part.pads, part.reference, at, rotation)
                    .map(Some),
                None => Ok(Some(d.place(symbol, at, rotation))),
            }
        });
        match placed {
            Ok(id) => {
                if labelled {
                    self.set_tool(Tool::Select);
                    // A wiring diagram's devices are named, not valued, and a power
                    // symbol (`#PWR`) names a net: neither is asked for a value.
                    self.value_prompt = id.filter(|id| !self.wiring() && self.document.components.iter()
                        .any(|c| c.id == *id && c.is_valued_part()));
                }
                id
            }
            Err(refusal) => {
                self.notice = Some(refusal);
                None
            }
        }
    }
    pub fn set_tool(&mut self, tool: Tool) {
        // A wiring diagram's connections are dragged pin to pin; it has no nets.
        if self.wiring()
            && matches!(
                tool,
                Tool::Wire | Tool::Junction | Tool::Label | Tool::NoConnect | Tool::PowerFlag | Tool::Power
            )
        {
            return;
        }
        self.tool = tool;
        self.wire_start = None;
        if tool != Tool::Place {
            self.placing_part = None;
        }
    }
    /// Why a connection may not start or end at this target: in a wiring diagram, a pin
    /// that already has its one connection.
    fn taken(&self, target: &ConnectionTarget) -> Option<String> {
        let ConnectionTarget::Terminal(terminal) = target else {
            return None;
        };
        let wire = self
            .document
            .attached_wire(terminal)
            .filter(|_| self.wiring())?;
        Some(format!(
            "{} already has connection {}. Each pin in a wiring diagram takes one connection.",
            self.document.terminal_name(terminal),
            wire.connection_id
        ))
    }
    pub fn undo(&mut self) {
        self.cancel();
        if self.history.can_undo() {
            self.history.undo(&mut self.document);
            self.selected = None;
            self.changed = true;
            self.note_change(None);
        }
    }
    pub fn redo(&mut self) {
        self.cancel();
        if self.history.can_redo() {
            self.history.redo(&mut self.document);
            self.selected = None;
            self.changed = true;
            self.note_change(None);
        }
    }
    pub fn cancel(&mut self) {
        self.terminal_drag = None;
        if let Some((before, _, _, _)) = self.path_drag.take() {
            self.document = before;
        }
        if let Some((before, _, _)) = self.drag.take() {
            self.document = before;
        }
        self.wire_start = None;
        self.tool = Tool::Select;
        self.cancel_board();
    }
    fn view_switch(&mut self, ui: &mut egui::Ui) {
        if self.wiring() {
            // Without nets there is nothing to lay out on a board.
            ui.label(egui::RichText::new("Wiring diagram").color(accent(ui)));
            ui.separator();
            return;
        }
        self.action_buttons(ui, ActionGroup::View);
        ui.separator();
    }
    pub fn toolbar(&mut self, ui: &mut egui::Ui) {
        if self.view == View::Board {
            return self.board_toolbar(ui);
        }
        let wiring = self.wiring();
        ui.horizontal_wrapped(|ui| {
            self.view_switch(ui);
            self.action_buttons(ui, ActionGroup::Tool);
            ui.separator();
            self.action_buttons(ui, ActionGroup::History);
            ui.separator();
            self.action_buttons(ui, ActionGroup::Zoom);
        });
        ui.horizontal_wrapped(|ui| {
            match self.tool {
                Tool::Place => {
                    ui.label("Click the sheet to place a symbol.");
                }
                Tool::Wire => {
                    ui.label("Click a pin or grid point, then click to draw. A wire ends on a pin, wire or junction; Escape stops drawing.");
                }
                Tool::Label => {
                    ui.label("Name:");
                    ui.add(egui::TextEdit::singleline(&mut self.label_text).desired_width(140.));
                    ui.label("Click a wire or pin to place.");
                }
                Tool::Move => {
                    ui.label("Click the new position. Attached wires follow.");
                }
                Tool::Junction => {
                    ui.label("Click a crossing to connect it; click the dot again to remove it.");
                }
                Tool::NoConnect => {
                    ui.label("Click a pin's end to mark it no-connect; click a marked pin again to take the mark off.");
                }
                Tool::PowerFlag => {
                    ui.label("Click a pin, or a wire of a net, to flag it driven from off the sheet (PWR_FLAG); click a flagged pin again to take the flag off.");
                }
                Tool::Power => {
                    ui.label("Power net:");
                    for name in POWER_NETS {
                        if ui.selectable_label(self.power_net_text == name, name).clicked() {
                            self.power_net_text = name.into();
                        }
                    }
                    ui.add(egui::TextEdit::singleline(&mut self.power_net_text).desired_width(80.));
                    ui.label("Click a pin, wire or grid point to place. Every symbol of one name is one net.");
                }
                Tool::Select if wiring => {
                    ui.label("Drag from pin to pin to connect; each pin takes one connection. Drag devices or wire segments to adjust. Right-drag pans.");
                    ui.label("Stock part for new connections:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.stock_part_number)
                            .desired_width(140.),
                    );
                }
                Tool::Select => {
                    ui.label("Drag between terminals or junctions to connect. Drag devices or wire segments to adjust. Right-drag pans.");
                }
            }
            self.action_buttons(ui, ActionGroup::Context);
        });
    }
    fn zoom_by(&mut self, factor: f32) {
        zoom_at(&mut self.zoom, &mut self.pan, factor, Vec2::ZERO);
    }
    pub fn library_panel(&mut self, ui: &mut egui::Ui) {
        self.library_hits.clear();
        if self.view == View::Board {
            return self.board_panel(ui);
        }
        ui.add_space(8.);
        // The tab this pane sits under says Assembly parts, and so does every
        // page and the canvas's own empty state: one name for one list.
        ui.heading("Assembly parts");
        // The way to fill this list, at the top of it whether it is empty or not.
        // While it is empty it is the pane's only action and is drawn as one:
        // an empty list that only SAYS where parts come from is a dead end.
        if let Some(label) = self.add_part_label.clone() {
            let empty = self.parts.is_empty();
            let text = egui::RichText::new(format!("+ {label}"));
            let button = if empty {
                egui::Button::new(text.strong().color(ui.visuals().selection.stroke.color))
                    .fill(ui.visuals().selection.bg_fill)
            } else {
                egui::Button::new(text)
            };
            let add = ui.add_sized(
                Vec2::new(ui.available_width(), if empty { 30. } else { 24. }),
                button,
            );
            self.library_hits.push(("parts:add".into(), add.rect));
            if add.clicked() {
                self.add_part_requested = true;
            }
            ui.add_space(4.);
        }
        // Named where the parts are, because a component whose device the assembly does
        // not offer is a broken reference to one of them, and the inspector's picker is
        // no use to a user who cannot find which component to select.
        let flagged: Vec<Uuid> = self.unlinked().into_iter().map(|(id, _)| id).collect();
        let unlinked: Vec<(Uuid, String)> = flagged
            .into_iter()
            .filter_map(|id| {
                self.document
                    .components
                    .iter()
                    .find(|c| c.id == id)
                    .map(|c| (id, c.reference.clone()))
            })
            .collect();
        if !unlinked.is_empty() {
            ui.label(
                egui::RichText::new("Unlinked")
                    .color(board_view::WARNING)
                    .strong(),
            );
            ui.small("These name a device this assembly does not offer. Select one to pick its device.");
            let mut chosen = None;
            for (id, reference) in &unlinked {
                if ui.button(reference).clicked() {
                    chosen = Some(*id);
                }
            }
            if let Some(id) = chosen {
                self.selected = Some(Selection::Component(id));
            }
            ui.separator();
        }
        if self.parts.is_empty() {
            ui.label(egui::RichText::new(&self.parts_hint).weak());
            return;
        }
        let search = ui.add(
            egui::TextEdit::singleline(&mut self.search)
                .hint_text("Search parts…")
                .desired_width(f32::INFINITY),
        );
        self.library_hits.push(("parts:search".into(), search.rect));
        let query = self.search.to_lowercase();
        let on_sheet: std::collections::BTreeSet<&str> = self
            .document
            .components
            .iter()
            .map(|c| c.reference.as_str())
            .collect();
        let entries: Vec<(&Part, bool)> = self
            .parts
            .iter()
            .filter(|p| {
                format!(
                    "{} {} {} {}",
                    p.name,
                    p.reference.as_deref().unwrap_or(""),
                    p.source.key,
                    p.symbol.description
                )
                .to_lowercase()
                .contains(&query)
            })
            .map(|p| {
                // A component drawn before components carried an occurrence has none,
                // and is bound by its part's key and its label, as the host binds it
                // (`ecad_parts::follow_sheet`). Read by occurrence alone, such a sheet
                // offered every part on it as unplaced, and a click was then refused.
                let placed = match &p.source.instance {
                    Some(instance) => self.document.components.iter().any(|c| {
                        c.part.as_ref().is_some_and(|source| match &source.instance {
                            Some(bound) => bound == instance,
                            None => source.key == p.source.key
                                && p.reference.as_deref() == Some(c.reference.as_str()),
                        })
                    }),
                    None => p.reference.as_deref().is_some_and(|r| on_sheet.contains(r)),
                };
                (p, placed)
            })
            .collect();
        let (mut chosen, mut open, mut dragging) = (None, None, false);
        let mut hits = Vec::new();
        egui::ScrollArea::vertical()
            .id_salt(("parts", &self.search))
            .max_height((ui.available_height() - 55.).max(80.))
            .show_rows(ui, 88., entries.len(), |ui, range| {
                for (part, placed) in &entries[range] {
                    let selected =
                        self.tool == Tool::Place && self.placing_part.as_ref() == Some(*part);
                    let response = part_card(ui, part, selected, *placed);
                    // By label: two parts of one name (ten 10k resistors) are two
                    // cards, and keyed by name the last would hide the rest.
                    let label = part.reference.as_deref().unwrap_or(&part.name);
                    hits.push((format!("parts:card:{label}"), response.rect));
                    response.context_menu(|ui| {
                        menu_ui(ui);
                        if ui.button("Open part").clicked() {
                            open = Some(part.source.key.clone());
                            ui.close();
                        }
                    });
                    if *placed {
                        continue;
                    }
                    if response.clicked() {
                        chosen = Some((*part).clone());
                    }
                    if response.drag_started() {
                        response.dnd_set_drag_payload(LibraryPartDrag((*part).clone()));
                        dragging = true;
                    }
                }
            });
        self.library_hits.extend(hits);
        if let Some(key) = open {
            self.open_part_requests.push(key);
        }
        if dragging {
            self.cancel();
        }
        if let Some(part) = chosen {
            self.begin_placing_part(part);
        }
        ui.separator();
        // Body size, weak, as the hint over an empty list is: this is the only
        // place the pane says how its cards are used.
        ui.label(
            egui::RichText::new(if self.wiring() {
                "Drag a part onto the sheet. Right-click one to open it."
            } else {
                "Drag a part onto the sheet. Right-click one to open it.\nRight-click a terminal to add a net flag."
            })
            .weak(),
        );
    }
    /// The whole inspector scrolls: a selected component's properties sit
    /// ABOVE the connection / net lists, and in a short pane they overflowed
    /// with only the list below them scrollable. Inside this area the inner
    /// lists grow to their content and scroll with everything else.
    pub fn inspector(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical()
            .id_salt("ecad-inspector")
            .auto_shrink([false, false])
            .show(ui, |ui| self.inspector_body(ui));
    }

    fn inspector_body(&mut self, ui: &mut egui::Ui) {
        if self.view == View::Board {
            return self.board_inspector(ui);
        }
        self.properties_hits.begin(ui.ctx());
        ui.add_space(12.);
        ui.heading("Properties");
        ui.add_space(12.);
        if let Some(Selection::Component(id)) = self.selected {
            if let Some(c) = self
                .document
                .components
                .iter()
                .find(|c| c.id == id)
                .cloned()
            {
                // A placed component's symbol is a copy of its part's, and the host
                // refreshes it from the part: it is edited on the part, never here.
                ui.label(egui::RichText::new(&c.symbol.library_id).color(accent(ui)));
                if !c.gates.is_empty() {
                    let names: Vec<String> = c.gates.iter().map(|g| c.gate_reference(g.gate)).collect();
                    ui.small(format!(
                        "One part in {} gates, {}: drag a gate to move it on its own, or \
                         Shift-drag to move the whole part. They share one reference, one \
                         footprint and one place on the board.",
                        names.len(),
                        names.join(", ")
                    ));
                }
                let mut reference = c.reference.clone();
                let mut value = c.value.clone();
                ui.label("Reference");
                // A label the host keeps is shown, never edited here.
                let host_label = self.labels_from_host.clone().filter(|_| c.part.is_some());
                let r = match &host_label {
                    Some(hint) => {
                        ui.label(egui::RichText::new(&reference).strong());
                        ui.small(hint);
                        false
                    }
                    None => ui.text_edit_singleline(&mut reference).changed(),
                };
                ui.label("Value");
                let v = ui.add(egui::TextEdit::singleline(&mut value).hint_text("10k, 100nF, LM358"));
                self.properties_hits.mark("value", &v);
                if self.value_prompt.take() == Some(id) {
                    v.request_focus();
                    let mut state = egui::TextEdit::load_state(ui.ctx(), v.id).unwrap_or_default();
                    let end = egui::text::CCursor::new(value.chars().count());
                    state.cursor.set_char_range(Some(egui::text::CCursorRange::two(egui::text::CCursor::new(0), end)));
                    egui::TextEdit::store_state(ui.ctx(), v.id, state);
                }
                let default = c.symbol.default_value();
                if !self.wiring() && c.is_valued_part() {
                    if c.value == default {
                        ui.small(egui::RichText::new(
                            "The symbol's own value. Type this part's, such as 10k: \
                             the BOM groups by it, and the netlist and pick-and-place write it.",
                        ).weak());
                    } else if !c.value.trim().is_empty() {
                        // The value typed here, offered to the parts placed from the same
                        // symbol that still read its value: four resistors are one edit.
                        let siblings = self.document.unvalued_siblings(id);
                        if !siblings.is_empty() {
                            let names: Vec<&str> = self.document.components.iter()
                                .filter(|o| siblings.contains(&o.id)).map(|o| o.reference.as_str()).collect();
                            let mut shown = names.iter().take(4).copied().collect::<Vec<_>>().join(", ");
                            if names.len() > 4 {
                                shown += &format!(" and {} more", names.len() - 4);
                            }
                            let b = ui.button(format!("{} for {shown} too", c.value)).on_hover_text(format!(
                                "{} placed from {} still read its value '{default}'. Give them '{}' as well, in one undo step.",
                                if names.len() == 1 { "This part is" } else { "These parts are" },
                                c.symbol.library_id, c.value
                            ));
                            self.properties_hits.mark("value_siblings", &b);
                            if b.clicked() {
                                let given = c.value.clone();
                                self.transaction(|d| {
                                    d.set_value(&siblings, &given);
                                });
                            }
                        }
                    }
                }
                if (r || v.changed())
                    && !reference.trim().is_empty()
                    && !self
                        .document
                        .components
                        .iter()
                        .any(|c| c.id != id && c.reference == reference)
                {
                    self.transaction_as(Some(format!("component:{id}:text")), |d| {
                        let c = d.components.iter_mut().find(|c| c.id == id).unwrap();
                        c.reference = reference;
                        c.value = value;
                    });
                }
                if ui.button("Move").clicked() {
                    self.set_tool(Tool::Move);
                }
                if ui.button("Rotate 90°").clicked() {
                    self.rotate();
                }
                if ui.button("Duplicate").clicked() {
                    self.duplicate();
                }
                ui.small("Connected wires follow moves and rotation.");
                if let Some(part) = &c.part {
                    ui.separator();
                    ui.label("Part");
                    ui.small(&part.key);
                    if ui
                        .button("Open part")
                        .on_hover_text("Its symbol and pads are edited on the part.")
                        .clicked()
                    {
                        self.open_part_requests.push(part.key.clone());
                    }
                    if let Some(lost) = self.unlinked_device(&c).map(str::to_owned) {
                        self.relink_picker(ui, id, &lost);
                    }
                }
                ui.separator();
                ui.label("Pins");
                for pin in &c.symbol.pins {
                    ui.small(format!("{} · {}", pin.number, pin.electrical_type));
                }
            }
        } else if let Some(Selection::Wire(id)) = self.selected {
            if self.wiring()
                && let Some(w) = self.document.wires.iter().find(|w| w.id == id).cloned()
            {
                let pin = |t: &Option<Terminal>| {
                    t.as_ref()
                        .map_or("?".into(), |t| self.document.terminal_name(t))
                };
                ui.label(
                    egui::RichText::new(format!("{} to {}", pin(&w.start), pin(&w.end)))
                        .color(accent(ui)),
                );
                let mut connection_id = w.connection_id.clone();
                let mut stock_part_number = w.stock_part_number.clone();
                ui.label("Connection ID");
                let c = ui.text_edit_singleline(&mut connection_id);
                ui.label("Stock part number");
                let s = ui.text_edit_singleline(&mut stock_part_number);
                if (c.changed() || s.changed())
                    && !connection_id.trim().is_empty()
                    && !self
                        .document
                        .wires
                        .iter()
                        .any(|w| w.id != id && w.connection_id == connection_id)
                {
                    self.transaction_as(Some(format!("wire:{id}:text")), |d| {
                        let w = d.wires.iter_mut().find(|w| w.id == id).unwrap();
                        w.connection_id = connection_id;
                        w.stock_part_number = stock_part_number;
                    });
                }
                ui.separator();
            }
            ui.label("Connection path");
            ui.small("Drag a segment or its square handle to move it. Endpoints stay attached.");
            if ui.button("Add jog").clicked() {
                let index = self.selected_segment;
                self.transaction(|d| {
                    if let Some(w) = d.wires.iter_mut().find(|w| w.id == id) {
                        w.add_jog(index);
                    }
                });
                self.selected_segment = index + 2;
            }
            let can_remove = self
                .document
                .wires
                .iter()
                .find(|w| w.id == id)
                .is_some_and(|w| w.removable_jog(self.selected_segment).is_some());
            if ui
                .add_enabled(can_remove, egui::Button::new("Remove jog"))
                .clicked()
            {
                let index = self.selected_segment;
                self.transaction(|d| {
                    if let Some(w) = d.wires.iter_mut().find(|w| w.id == id) {
                        w.remove_jog(index);
                    }
                });
                self.selected_segment = 0;
            }
        } else if let Some(Selection::Label(id)) = self.selected {
            if let Some(label) = self.document.labels.iter().find(|l| l.id == id) {
                let mut name = label.name.clone();
                ui.label("Net name");
                if ui.text_edit_singleline(&mut name).changed() && !name.trim().is_empty() {
                    self.transaction_as(Some(format!("label:{id}:name")), |d| {
                        d.labels.iter_mut().find(|l| l.id == id).unwrap().name = name.trim().into()
                    });
                }
                if ui.button("Move").clicked() {
                    self.set_tool(Tool::Move);
                }
                if ui.button("Rotate 90°").clicked() {
                    self.rotate();
                }
                ui.small("Drag the body to move. Drag from the tip to draw a wire.");
            }
        } else {
            ui.label("Select a device or connection to edit it.");
            if !self.wiring() {
                self.values_table(ui);
            }
        }
        if self.selected.is_some() && ui.button("Delete selection").clicked() {
            self.delete();
        }
        ui.add_space(16.);
        ui.separator();
        if self.wiring() {
            ui.heading("Connections");
            let connections = self.document.connections();
            ui.label(format!(
                "{} components · {} connections",
                self.document.components.len(),
                connections.len()
            ));
            egui::ScrollArea::vertical()
                .id_salt("connections")
                .show(ui, |ui| {
                    for c in &connections {
                        ui.small(format!(
                            "{}  {}.{} to {}.{}  {}",
                            c.connection_id,
                            c.from_ref_des,
                            c.from_port,
                            c.to_ref_des,
                            c.to_port,
                            c.stock_part_number
                        ));
                    }
                });
            return;
        }
        ui.heading("Connectivity");
        let netlist = self.document.netlist();
        ui.label(format!(
            "{} components · {} nets",
            self.document.components.len(),
            netlist.nets.len()
        ));
        self.erc_section(ui);
        ui.add_space(8.);
        ui.label(egui::RichText::new("Nets").strong());
        egui::ScrollArea::vertical().id_salt("nets").show(ui, |ui| {
            for net in &netlist.nets {
                ui.collapsing(format!("{}  ·  {} pins", net.name, net.pins.len()), |ui| {
                    for pin in &net.pins {
                        ui.small(format!("{}.{}", pin.reference, pin.number));
                    }
                });
            }
        });
    }
    /// Every part's value in one table, as KiCad's Symbol Fields Table gives them:
    /// a part the host seats on its own (Add Component on the PCB workbench) is never
    /// placed from the palette, so this is where its value is typed, and four
    /// resistors are four fields in a column rather than four selections (the third
    /// eCAD audit, B3). A value still the symbol's own is drawn weak.
    fn values_table(&mut self, ui: &mut egui::Ui) {
        let mut rows: Vec<(Uuid, String, String, bool)> = self
            .document
            .components
            .iter()
            .filter(|c| c.is_valued_part())
            .map(|c| (c.id, c.reference.clone(), c.value.clone(), c.value == c.symbol.default_value()))
            .collect();
        if rows.is_empty() {
            return;
        }
        rows.sort_by(|a, b| brep_ecad_core::footprint::natural_cmp(&a.1, &b.1));
        let unvalued = rows.iter().filter(|r| r.3).count();
        ui.add_space(12.);
        ui.label(egui::RichText::new("Values").strong());
        ui.small(match unvalued {
            0 => "Each part has its own value.".to_owned(),
            n => format!(
                "{n} of {} have no value of their own yet (grey). Type each part's, such as \
                 10k: the BOM groups by it, and the netlist and pick-and-place write it.",
                rows.len()
            ),
        });
        let weak = ui.visuals().weak_text_color();
        egui::ScrollArea::vertical()
            .id_salt("values")
            .max_height(260.)
            .show(ui, |ui| {
                egui::Grid::new("values").num_columns(2).striped(true).show(ui, |ui| {
                    for (id, reference, mut value, default) in rows {
                        ui.label(&reference);
                        let mut field =
                            egui::TextEdit::singleline(&mut value).desired_width(120.).hint_text("10k");
                        if default {
                            field = field.text_color(weak);
                        }
                        let field = ui.add(field);
                        self.properties_hits.mark(format!("value:{reference}"), &field);
                        if field.changed() {
                            self.transaction_as(Some(format!("component:{id}:text")), |d| {
                                d.set_value(&[id], &value);
                            });
                        }
                        ui.end_row();
                    }
                });
            });
    }
    fn rotate(&mut self) {
        if self.tool == Tool::Place {
            self.rotation = (self.rotation + 1) % 4;
        } else if let Some(Selection::Component(id)) = self.selected {
            let gate = self.selected_gate();
            self.transaction(|d| Self::move_component(d, id, gate, None, Some(1)));
        } else if let Some(Selection::Label(id)) = self.selected {
            self.transaction(|d| {
                if let Some(l) = d.labels.iter_mut().find(|l| l.id == id) {
                    l.rotation = (l.rotation + 1) % 4;
                }
            });
        }
    }
    /// Move component `id` to `at` and turn it by `turn` more quarter turns,
    /// or only the one gate `gate` of it; `None` leaves that alone.
    fn move_component(d: &mut Document, id: Uuid, gate: Option<u32>, at: Option<Point>, turn: Option<u8>) {
        let Some(c) = d.components.iter().find(|c| c.id == id) else {
            return;
        };
        let (old_at, old_rotation) = match gate.and_then(|g| c.gate(g)) {
            Some(g) => (g.at, g.rotation),
            None => (c.at, c.rotation),
        };
        let at = at.unwrap_or(old_at);
        let rotation = (old_rotation + turn.unwrap_or(0)) % 4;
        match gate {
            Some(gate) => d.transform_gate(id, gate, at, rotation),
            None => d.transform_component(id, at, rotation),
        }
    }
    /// Place another of the selected component. For a component placed from a part
    /// that is the next one of that part the host offers that is not on the sheet yet.
    fn duplicate(&mut self) {
        let Some(Selection::Component(id)) = self.selected else {
            return;
        };
        let Some(c) = self.document.components.iter().find(|c| c.id == id) else {
            return;
        };
        let rotation = c.rotation;
        let Some(source) = &c.part else {
            self.placing = Some(c.symbol.clone());
            self.set_tool(Tool::Place);
            self.rotation = rotation;
            return;
        };
        let on_sheet = |r: &String| self.document.components.iter().any(|c| &c.reference == r);
        let next = self
            .parts
            .iter()
            .find(|p| p.source.key == source.key && !p.reference.as_ref().is_some_and(on_sheet))
            .cloned();
        match next {
            Some(part) => {
                self.begin_placing_part(part);
                self.rotation = rotation;
            }
            None => {
                self.notice = Some(format!(
                    "Every {} in the assembly is already on the sheet.",
                    c.value
                ))
            }
        }
    }
    fn delete(&mut self) {
        if let Some(s) = self.selected.take() {
            self.transaction(|d| match s {
                Selection::Component(id) => d.delete_component(id),
                Selection::Wire(id) => d.wires.retain(|w| w.id != id),
                Selection::Label(id) => d.labels.retain(|l| l.id != id),
                Selection::Junction(p) => {
                    d.junctions.remove(&p);
                }
            });
        }
    }
    fn fit(&mut self, size: Vec2) {
        let mut points: Vec<_> = self
            .document
            .components
            .iter()
            .flat_map(|c| {
                c.symbol
                    .pins
                    .iter()
                    .map(|p| c.pin_at(p))
                    .chain(std::iter::once(c.at))
                    .chain(c.gates.iter().map(|g| g.at))
            })
            .collect();
        points.extend(self.document.wires.iter().flat_map(|w| w.points()));
        points.extend(self.document.labels.iter().map(|l| l.at));
        if points.is_empty() {
            self.pan = Vec2::ZERO;
            self.zoom = 0.012;
            return;
        }
        let min = Point::new(
            points.iter().map(|p| p.x).min().unwrap() - 10000,
            points.iter().map(|p| p.y).min().unwrap() - 10000,
        );
        // The right side also holds each part's reference and value.
        let max = Point::new(
            points.iter().map(|p| p.x).max().unwrap() + 17780,
            points.iter().map(|p| p.y).max().unwrap() + 10000,
        );
        self.zoom = valid_zoom(
            (size.x / (max.x - min.x) as f32)
                .min(size.y / (max.y - min.y) as f32)
                .min(0.03),
            self.zoom,
        );
        self.pan = -Vec2::new((min.x + max.x) as f32 / 2., (min.y + max.y) as f32 / 2.) * self.zoom;
    }
    fn screen(&self, p: Point, rect: Rect) -> Pos2 {
        rect.center() + self.pan + Vec2::new(p.x as f32, p.y as f32) * self.zoom
    }
    fn world(&self, p: Pos2, rect: Rect) -> Point {
        let v = (p - rect.center() - self.pan) / self.zoom;
        Point::new(v.x.round() as i32, v.y.round() as i32)
    }
    fn snap(&self, p: Pos2, rect: Rect) -> Point {
        let mut candidates: Vec<_> = self
            .document
            .components
            .iter()
            .flat_map(|c| c.symbol.pins.iter().map(|pin| c.pin_at(pin)))
            .collect();
        candidates.extend(self.document.wires.iter().flat_map(|w| w.points()));
        candidates.extend(self.document.junctions.iter().copied());
        candidates
            .into_iter()
            .filter_map(|q| {
                let d = self.screen(q, rect).distance(p);
                (d < 10.).then_some((q, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map_or_else(|| self.world(p, rect).snapped(), |v| v.0)
    }
    /// Whether `l` is the net name a power symbol gives its own pin. The symbol
    /// draws that name itself, so the flag is neither drawn nor picked: a click
    /// there picks the symbol.
    fn names_power_symbol(&self, l: &Label) -> bool {
        l.terminal.as_ref().is_some_and(|t| {
            self.document
                .components
                .iter()
                .any(|c| c.id == t.component && c.symbol.power_net.is_some())
        })
    }
    fn hit(&self, p: Pos2, rect: Rect, ctx: &egui::Context) -> Option<Selection> {
        for l in self.document.labels.iter().rev() {
            if self.names_power_symbol(l) {
                continue;
            }
            let tip = self.screen(l.at, rect);
            if l.flag || l.terminal.is_some() {
                let (_, outline, _) =
                    net_flag_layout(ctx, &l.name, tip, self.zoom / 0.012, l.rotation);
                if Rect::from_points(&outline).contains(p) {
                    return Some(Selection::Label(l.id));
                }
            } else if tip.distance(p) < 14. {
                return Some(Selection::Label(l.id));
            }
        }
        for j in &self.document.junctions {
            if self.screen(*j, rect).distance(p) < 7. {
                return Some(Selection::Junction(*j));
            }
        }
        for c in self.document.components.iter().rev() {
            if self
                .drawn_gates(c, rect)
                .iter()
                .any(|(_, bounds)| bounds.expand(7.).contains(p))
            {
                return Some(Selection::Component(c.id));
            }
        }
        self.hit_segment(p, rect).map(|(id, _)| Selection::Wire(id))
    }
    fn hit_target(&self, p: Pos2, rect: Rect) -> Option<ConnectionTarget> {
        if let Some(t) = self.hit_terminal(p, rect) {
            return Some(ConnectionTarget::Terminal(t));
        }
        self.document
            .junctions
            .iter()
            .chain(self.document.labels.iter().map(|l| &l.at))
            .filter_map(|q| {
                let distance = self.screen(*q, rect).distance(p);
                (distance <= 12.).then_some((*q, distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(q, _)| ConnectionTarget::Junction(q))
    }
    /// Whether a label was placed since the last call. A click on the sheet takes
    /// the keyboard from the host's name field, so without this the next name typed
    /// went to the canvas and the next click silently repeated the last name (the
    /// 2026-09-23 audit's VCC placed twice, shorting GND). The host answers by
    /// focusing the field again with its text selected: typing replaces the name,
    /// another click repeats it.
    pub fn take_label_placed(&mut self) -> bool {
        std::mem::take(&mut self.label_placed)
    }
    /// What a label named `name` at `at` would join, when it joins two nets that
    /// were already distinct and each reach a pin: the one under it and the one the
    /// name already names. The line says so, pins and all, because joining by name
    /// is the label's job and a mistyped or repeated name does it just as quietly.
    ///
    /// `true` with it is a WARNING: two nets that each already had a NAME are now
    /// one, which is what a mistyped name does (VCC on the GND net). Joining a
    /// pin or an unnamed net to the net a name names is what a label is placed
    /// FOR, and says so without the alarm: the 2026-09-23 re-audit counted 12 of
    /// its 18 intended placements drawing the red line.
    fn label_joins(&self, what: &str, name: &str, at: Point) -> Option<(String, bool)> {
        let before = self.document.netlist();
        let mut after = self.document.clone();
        after.labels.push(Label {
            flag: false,
            rotation: 0,
            terminal: None,
            id: Uuid::nil(),
            name: name.to_owned(),
            at,
        });
        let after = after.netlist();
        let net = after.nets.iter().find(|n| n.points.contains(&at))?;
        let joined: Vec<_> = before
            .nets
            .iter()
            .filter(|b| {
                b.pins.iter().any(|p| {
                    net.pins
                        .iter()
                        .any(|q| q.component_id == p.component_id && q.number == p.number)
                })
            })
            .collect();
        if joined.len() < 2 {
            return None;
        }
        let named = joined.iter().filter(|n| !n.labels.is_empty()).count();
        let describe = |n: &brep_ecad_core::Net| {
            let mut pins: Vec<_> = n
                .pins
                .iter()
                .take(3)
                .map(|p| format!("{}.{}", p.reference, p.number))
                .collect();
            if n.pins.len() > 3 {
                pins.push("…".into());
            }
            format!("{} ({})", n.name, pins.join(", "))
        };
        let nets = joined.iter().map(|n| describe(n)).collect::<Vec<_>>().join(" and ");
        Some(if named >= 2 {
            (
                format!(
                    "{what} {name} joined {nets}, two named nets, into one net of {} pins. Undo if that was not meant.",
                    net.pins.len()
                ),
                true,
            )
        } else {
            (format!("{what} {name} joined {nets} into one net of {} pins.", net.pins.len()), false)
        })
    }
    /// Mark `terminal` no-connect, or take its mark off. A mark on a pin that is
    /// joined is kept, and said: the check reports it until one or the other goes.
    fn toggle_no_connect(&mut self, terminal: Terminal) {
        let name = self.document.terminal_name(&terminal);
        let marked = self.document.no_connects.contains(&terminal);
        self.transaction(|d| {
            if marked {
                d.no_connects.retain(|t| *t != terminal);
            } else {
                d.no_connects.push(terminal.clone());
            }
        });
        if marked {
            self.note = Some(format!("{name}: no-connect taken off."));
            return;
        }
        let joined = self.document.netlist().nets.into_iter().find(|n| {
            n.pins.len() > 1
                && n.pins.iter().any(|p| p.component_id == terminal.component && p.number == terminal.pin)
        });
        match joined {
            Some(net) => {
                self.notice = Some(format!(
                    "{name} is marked no-connect, but it is joined to {}: the check reports the mark until the wire or the mark goes.",
                    net.name
                ))
            }
            None => self.note = Some(format!("{name} marked no-connect: the check does not ask for it to be joined.")),
        }
    }
    /// Put a power flag on `terminal`, or take it off.
    fn toggle_power_flag(&mut self, terminal: Terminal) {
        let name = self.document.terminal_name(&terminal);
        let flagged = self.document.power_flags.contains(&terminal);
        self.transaction(|d| {
            if flagged {
                d.power_flags.retain(|t| *t != terminal);
            } else {
                d.power_flags.push(terminal.clone());
            }
        });
        let net = self
            .document
            .netlist()
            .nets
            .into_iter()
            .find(|n| n.pins.iter().any(|p| p.component_id == terminal.component && p.number == terminal.pin))
            .map_or_else(String::new, |n| n.name);
        self.note = Some(if flagged {
            format!("{name}: power flag taken off {net}.")
        } else {
            format!("Power flag on {name}: {net} is driven from off the sheet.")
        });
    }
    /// The pin the Power flag tool flags for a click at `p` (snapped to `q`): the pin
    /// under it, or, on a wire, label or junction of a net, that net's first power
    /// input, or its first pin when it has none. A flag marks the NET, and the net
    /// is what the check reads.
    fn power_flag_target(&self, p: Pos2, q: Point, rect: Rect) -> Option<Terminal> {
        if let Some(t) = self.hit_terminal(p, rect) {
            return Some(t);
        }
        if !self.lands_on_net(q) {
            return None;
        }
        // A point along a wire is not one of the net's points; the wire's end is.
        let anchor = self
            .document
            .wires
            .iter()
            .find(|w| w.points().windows(2).any(|s| brep_ecad_core::on_segment(q, s[0], s[1])))
            .map_or(q, |w| w.a);
        let net = self.document.netlist().nets.into_iter().find(|n| n.points.contains(&anchor))?;
        // A pin already flagged on the net is the one a second click takes off.
        let pin = net
            .pins
            .iter()
            .find(|p| {
                self.document
                    .power_flags
                    .iter()
                    .any(|t| t.component == p.component_id && t.pin == p.number)
            })
            .or_else(|| net.pins.iter().find(|p| p.electrical_type == "power_in"))
            .or_else(|| net.pins.first())?;
        Some(Terminal {
            component: pin.component_id,
            pin: pin.number.clone(),
        })
    }
    /// Place the Power tool's symbol at `q`, saying what it joined.
    fn place_power_symbol(&mut self, q: Point) {
        let name = self.power_net_text.trim().to_owned();
        if name.is_empty() {
            self.notice = Some("Choose or type a power net name first.".into());
            return;
        }
        let joined = self.label_joins("Power symbol", &name, q);
        let mut id = None;
        self.transaction(|d| id = Some(d.place(power_symbol(&name), q, 0)));
        self.selected = id.map(Selection::Component);
        match joined {
            Some((line, true)) => self.notice = Some(line),
            Some((line, false)) => self.note = Some(line),
            None => {
                let reference = id
                    .and_then(|id| self.document.components.iter().find(|c| c.id == id))
                    .map(|c| c.reference.clone())
                    .unwrap_or_default();
                self.note = Some(format!("{reference} placed on {name}."));
            }
        }
    }
    /// Whether `q` is already a point of the net: a pin, a junction, a label, or
    /// anywhere along a wire, which is where [`Tool::Wire`] ends a run.
    fn lands_on_net(&self, q: Point) -> bool {
        let d = &self.document;
        d.terminal_at(q).is_some()
            || d.junctions.contains(&q)
            || d.labels.iter().any(|l| l.at == q)
            || d.wires.iter().any(|w| {
                w.points()
                    .windows(2)
                    .any(|s| brep_ecad_core::on_segment(q, s[0], s[1]))
            })
    }
    fn hit_terminal(&self, p: Pos2, rect: Rect) -> Option<Terminal> {
        self.document
            .components
            .iter()
            .flat_map(|c| c.symbol.pins.iter().map(move |pin| (c, pin)))
            .filter_map(|(c, pin)| {
                let distance = self.screen(c.pin_at(pin), rect).distance(p);
                (distance <= 12.).then_some((
                    Terminal {
                        component: c.id,
                        pin: pin.number.clone(),
                    },
                    distance,
                ))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(terminal, _)| terminal)
    }
    fn hit_segment(&self, p: Pos2, rect: Rect) -> Option<(Uuid, usize)> {
        self.document
            .wires
            .iter()
            .flat_map(|w| {
                w.points()
                    .windows(2)
                    .enumerate()
                    .map(|(i, s)| {
                        (
                            w.id,
                            i,
                            segment_distance(p, self.screen(s[0], rect), self.screen(s[1], rect)),
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .filter(|(_, _, d)| *d < 9.)
            .min_by(|a, b| a.2.total_cmp(&b.2))
            .map(|(id, i, _)| (id, i))
    }
    fn bounds(&self, c: &Component, rect: Rect) -> Rect {
        self.gate_bounds(c, None, rect)
    }
    /// The screen box of one gate of `c` ([`Component::gates`]), or of all of it
    /// for `None`. A component drawn whole has one box whichever is asked.
    fn gate_bounds(&self, c: &Component, gate: Option<u32>, rect: Rect) -> Rect {
        let drawn = |g: u32| c.gates.is_empty() || gate.is_none_or(|gate| gate == g);
        let at = gate.and_then(|g| c.gate(g)).map_or(c.at, |g| g.at);
        let mut r = Rect::from_center_size(self.screen(at, rect), Vec2::splat(8.));
        for (i, g) in c.symbol.graphics.iter().enumerate() {
            if !drawn(c.symbol.graphic_gate(i)) {
                continue;
            }
            match g {
                Graphic::Text { at, .. } => {
                    r.extend_with(self.screen(c.graphic_transform(i, *at), rect));
                }
                Graphic::Path(points) => {
                    for p in points {
                        r.extend_with(self.screen(c.graphic_transform(i, *p), rect));
                    }
                }
                Graphic::Circle { center, radius } => {
                    let center = self.screen(c.graphic_transform(i, *center), rect);
                    r = r.union(Rect::from_center_size(
                        center,
                        Vec2::splat(*radius as f32 * self.zoom * 2.),
                    ));
                }
            }
        }
        for p in c.symbol.pins.iter().filter(|p| drawn(p.gate)) {
            r.extend_with(self.screen(c.pin_at(p), rect));
        }
        r
    }
    /// The gates `c` is drawn as, each with its box: `(gate, box)`, gate `0` for
    /// a component drawn whole.
    fn drawn_gates(&self, c: &Component, rect: Rect) -> Vec<(u32, Rect)> {
        if c.gates.is_empty() {
            vec![(0, self.bounds(c, rect))]
        } else {
            c.gates
                .iter()
                .map(|g| (g.gate, self.gate_bounds(c, Some(g.gate), rect)))
                .collect()
        }
    }
    /// The gate of the selected component that selection picked, when it places
    /// its gates apart: what a drag, Move or Rotate acts on.
    fn selected_gate(&self) -> Option<u32> {
        match (self.selected, self.selected_gate) {
            (Some(Selection::Component(id)), Some((gid, gate))) if id == gid => self
                .document
                .components
                .iter()
                .find(|c| c.id == id)
                .and_then(|c| c.gate(gate))
                .map(|g| g.gate),
            _ => None,
        }
    }
    /// Select what is under `p`, remembering which gate of a component it was.
    /// With Shift held no gate is remembered: the whole part is picked, and a
    /// drag, Move or Rotate then acts on every gate of it together.
    fn pick(&mut self, p: Pos2, rect: Rect, ctx: &egui::Context) {
        self.selected = self.hit(p, rect, ctx);
        let whole = ctx.input(|i| i.modifiers.shift);
        self.selected_gate = match self.selected {
            Some(Selection::Component(id)) if !whole => self.hit_gate(id, p, rect).map(|g| (id, g)),
            _ => None,
        };
    }
    /// Which gate of component `id` is under `p`, when it places its gates apart.
    fn hit_gate(&self, id: Uuid, p: Pos2, rect: Rect) -> Option<u32> {
        let c = self.document.components.iter().find(|c| c.id == id)?;
        self.drawn_gates(c, rect)
            .into_iter()
            .rev()
            .find(|(g, r)| *g > 0 && r.expand(7.).contains(p))
            .map(|(g, _)| g)
    }
    /// Where each clickable thing of the view in front is drawn, in screen points,
    /// keyed for a host that drives the editor by key rather than by coordinate.
    /// `canvas` is the rect [`Editor::show`] was given; the board keeps its own.
    /// Computed with the same mapping and bounds the canvas paints and hit-tests
    /// with, so the centre of each rect is a point the editor answers for that
    /// thing. On the sheet: `component:<reference>` (its bounds),
    /// `pin:<reference>.<pin>` (the pin's tip), `wire:<index>` (the middle of its
    /// longest segment), `label:<index>`, `junction:<index>`, and the Connectivity
    /// panel's check, while it is drawn: `inspector:erc:run`, `inspector:erc:row:<i>`
    /// and `inspector:erc:fix:<i>`. On the board:
    /// `part:<reference>` (its courtyard), `pad:<reference>.<number>` (one pad as
    /// drawn; a mechanical pad has no number and publishes none), `track:<index>`,
    /// `via:<index>`.
    pub fn hits(&self, canvas: Rect) -> Vec<(String, Rect)> {
        if self.view == View::Board {
            return self.board_hits();
        }
        let mut out = Vec::new();
        for c in &self.document.components {
            // A component whose gates are placed apart is one rect per gate, by
            // the name the sheet gives it (`component:U1B`), and `component:U1`
            // is its first gate: a point the editor answers with U1, where the
            // box around every gate may hold nothing at all. So a click there
            // picks the part and a drag moves gate A, as a drag of U1A does; a
            // Shift-drag from any gate's rect moves the whole part.
            let gates = self.drawn_gates(c, canvas);
            out.push((format!("component:{}", c.reference), gates[0].1));
            for (gate, bounds) in gates.iter().filter(|(g, _)| *g > 0) {
                out.push((format!("component:{}", c.gate_reference(*gate)), *bounds));
            }
            for pin in &c.symbol.pins {
                let tip = self.screen(c.pin_at(pin), canvas);
                out.push((format!("pin:{}.{}", c.reference, pin.number), Rect::from_center_size(tip, Vec2::splat(12.))));
            }
        }
        for (i, w) in self.document.wires.iter().enumerate() {
            let points = w.points();
            let longest = points
                .windows(2)
                .map(|s| (self.screen(s[0], canvas), self.screen(s[1], canvas)))
                .max_by(|a, b| a.0.distance(a.1).total_cmp(&b.0.distance(b.1)));
            if let Some((a, b)) = longest {
                out.push((format!("wire:{i}"), Rect::from_center_size(a.lerp(b, 0.5), Vec2::splat(8.))));
            }
        }
        for (i, l) in self.document.labels.iter().enumerate() {
            out.push((format!("label:{i}"), Rect::from_center_size(self.screen(l.at, canvas), Vec2::splat(12.))));
        }
        for (i, j) in self.document.junctions.iter().enumerate() {
            out.push((format!("junction:{i}"), Rect::from_center_size(self.screen(*j, canvas), Vec2::splat(8.))));
        }
        // The markers, by the pin they mark, where each is drawn.
        for (prefix, marks) in [("no_connect", &self.document.no_connects), ("power_flag", &self.document.power_flags)] {
            for t in marks {
                if let Some(at) = self.document.terminal_position(t) {
                    let r = Rect::from_center_size(self.screen(at, canvas), Vec2::splat(12.));
                    out.push((format!("{prefix}:{}", self.document.terminal_name(t)), r));
                }
            }
        }
        out.extend(self.erc.hits.shown(self.canvas_pass));
        out.extend(self.properties_hits.shown(self.canvas_pass));
        out
    }
    /// Draw and interact inside any egui Ui; no global panels or file dialogs.
    pub fn show(&mut self, ui: &mut egui::Ui) {
        // Before anything is drawn, because a device the HOST added reaches the editor
        // as a change to `parts` and never as an edit here. Gated; see
        // [`Editor::follow_parts`].
        self.follow_parts();
        if self.view == View::Board {
            return self.show_board(ui);
        }
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        self.canvas_size = rect.size();
        self.canvas_pass = ui.ctx().cumulative_pass_nr();
        let pointer = ui.input(|i| i.pointer.hover_pos());
        if response.hovered() && ui.input(|i| i.pointer.primary_pressed()) {
            self.notice = None;
            self.note = None;
            // A value prompt the Inspector never drew (its tab behind another) dies
            // here rather than taking the keys whenever the tab comes forward.
            self.value_prompt = None;
        }
        if let Some(payload) = response.dnd_release_payload::<LibraryPartDrag>()
            && let Some(p) = pointer
        {
            let at = self.snap(p, rect);
            self.placing_part = Some(payload.0.clone());
            let id = self.place_here(payload.0.symbol.clone(), at, 0);
            self.selected = id.map(Selection::Component);
            self.set_tool(Tool::Select);
        }
        // The wheel zooms about the pointer, so it needs the pointer here. The keys
        // do not: see [`keys_reach_editor`].
        if response.hovered() && !ui.ctx().egui_wants_keyboard_input() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.
                && let Some(p) = pointer
            {
                zoom_at(&mut self.zoom, &mut self.pan, (scroll * 0.002).exp(), p - rect.center());
            }
        }
        if keys_reach_editor(ui.ctx()) {
            self.run_shortcuts(ui.ctx());
        }
        if response.dragged_by(PointerButton::Middle)
            || response.dragged_by(PointerButton::Secondary)
        {
            self.pan += ui.input(|i| i.pointer.delta());
        }
        if response.secondary_clicked() {
            self.cancel();
            self.context_terminal = pointer.and_then(|p| self.hit_terminal(p, rect));
            match pointer {
                Some(p) => self.pick(p, rect, ui.ctx()),
                None => self.selected = None,
            }
        }
        response.context_menu(|ui| {
            menu_ui(ui);
            if matches!(
                self.selected,
                Some(Selection::Component(_) | Selection::Label(_))
            ) && ui.button("Rotate 90°").clicked()
            {
                self.rotate();
                ui.close();
            }
            if self.selected.is_some() && ui.button("Delete").clicked() {
                self.delete();
                ui.close();
            }
            if !self.wiring()
                && let Some(terminal) = self.context_terminal.clone()
            {
                let existing = self
                    .document
                    .labels
                    .iter()
                    .find(|l| l.terminal.as_ref() == Some(&terminal));
                if ui
                    .button(if existing.is_some() {
                        "Edit net flag…"
                    } else {
                        "Add net flag…"
                    })
                    .clicked()
                {
                    self.net_flag_edit = Some(NetFlagEdit {
                        terminal: terminal.clone(),
                        name: existing.map_or(String::new(), |l| l.name.clone()),
                        focus: true,
                    });
                    ui.close();
                }
                if existing.is_some() && ui.button("Remove net flag").clicked() {
                    self.transaction(|d| {
                        d.labels.retain(|l| l.terminal.as_ref() != Some(&terminal))
                    });
                    ui.close();
                }
            }
        });
        if matches!(self.tool, Tool::Select | Tool::Wire)
            && response.drag_started_by(PointerButton::Primary)
            && let Some(p) = ui.input(|i| i.pointer.press_origin())
        {
            if let Some(terminal) = self.hit_target(p, rect) {
                if let Some(refusal) = self.taken(&terminal) {
                    self.notice = Some(refusal);
                } else {
                    self.wire_start = None;
                    self.terminal_drag = Some(terminal);
                }
            } else if self.tool == Tool::Select {
                self.pick(p, rect, ui.ctx());
                match self.selected {
                    Some(Selection::Component(id)) => {
                        if let Some(c) = self.document.components.iter().find(|c| c.id == id) {
                            let at = self.selected_gate().and_then(|g| c.gate(g)).map_or(c.at, |g| g.at);
                            self.drag = Some((self.document.clone(), self.world(p, rect), at));
                        }
                    }
                    Some(Selection::Label(id)) => {
                        if let Some(l) = self.document.labels.iter().find(|l| l.id == id) {
                            self.drag = Some((self.document.clone(), self.world(p, rect), l.at));
                        }
                    }
                    Some(Selection::Wire(id)) => {
                        if let Some((_, index)) = self.hit_segment(p, rect) {
                            self.selected_segment = index;
                            self.path_drag =
                                Some((self.document.clone(), id, index, self.world(p, rect)));
                        }
                    }
                    _ => {}
                }
            }
        }
        if let (Some((before, start, at)), Some(p), Some(Selection::Component(id))) =
            (&self.drag, pointer, self.selected)
        {
            let q = self.world(p, rect);
            let target = Point::new(at.x + q.x - start.x, at.y + q.y - start.y).snapped();
            let gate = self.selected_gate();
            self.document = before.clone();
            Self::move_component(&mut self.document, id, gate, Some(target), None);
        }
        if let (Some((before, start, at)), Some(p), Some(Selection::Label(id))) =
            (&self.drag, pointer, self.selected)
        {
            let q = self.world(p, rect);
            let target_screen =
                self.screen(Point::new(at.x + q.x - start.x, at.y + q.y - start.y), rect);
            self.document = before.clone();
            let target = self.snap(target_screen, rect);
            let rotation = self
                .document
                .labels
                .iter()
                .find(|l| l.id == id)
                .map_or(0, |l| l.rotation);
            self.document.transform_label(id, target, rotation);
        }
        if let (Some((before, id, index, start)), Some(p)) = (&self.path_drag, pointer) {
            let q = self.world(p, rect);
            let delta = Point::new(q.x - start.x, q.y - start.y).snapped();
            self.document = before.clone();
            if let Some(w) = self.document.wires.iter_mut().find(|w| w.id == *id) {
                w.slide_segment(*index, delta);
            }
        }
        if response.drag_stopped_by(PointerButton::Primary) {
            if let Some(start) = self.terminal_drag.take()
                && let Some(end) = pointer
                    .filter(|p| rect.contains(*p))
                    .and_then(|p| self.hit_target(p, rect))
            {
                if let Some(refusal) = self.taken(&end) {
                    self.notice = Some(refusal);
                } else {
                    let mut id = None;
                    let stock_part_number = self.stock_part_number.trim().to_owned();
                    self.transaction(|d| {
                        id = d.connect_targets(start, end);
                        if d.kind == DocumentKind::Wiring
                            && let Some(w) = d.wires.iter_mut().find(|w| Some(w.id) == id)
                        {
                            w.stock_part_number = stock_part_number;
                        }
                    });
                    if let Some(id) = id {
                        self.selected = Some(Selection::Wire(id));
                        self.selected_segment = 0;
                    }
                }
            }
            if let Some((before, _, _)) = self.drag.take() {
                self.commit(before, None);
            }
            if let Some((before, _, _, _)) = self.path_drag.take() {
                self.commit(before, None);
            }
        }
        if response.clicked_by(PointerButton::Primary)
            && let Some(p) = pointer
        {
            let q = self.snap(p, rect);
            match self.tool {
                Tool::Select => {
                    self.pick(p, rect, ui.ctx());
                    if let Some((_, index)) = self.hit_segment(p, rect) {
                        self.selected_segment = index;
                    }
                }
                Tool::Move => {
                    if let Some(Selection::Component(id)) = self.selected {
                        let gate = self.selected_gate();
                        self.transaction(|d| Self::move_component(d, id, gate, Some(q), None));
                    }
                    if let Some(Selection::Label(id)) = self.selected {
                        self.transaction(|d| {
                            if let Some(l) = d.labels.iter().find(|l| l.id == id) {
                                let rotation = l.rotation;
                                d.transform_label(id, q, rotation);
                            }
                        });
                    }
                    self.set_tool(Tool::Select);
                }
                Tool::Place => {
                    if let Some(s) = self.placing.clone() {
                        let rotation = self.rotation;
                        let id = self.place_here(s, q, rotation);
                        self.selected = id.map(Selection::Component);
                    }
                }
                Tool::Junction => {
                    self.transaction(|d| {
                        if !d.junctions.remove(&q) {
                            d.junctions.insert(q);
                        }
                    });
                }
                Tool::NoConnect => match self.hit_terminal(p, rect) {
                    Some(terminal) => self.toggle_no_connect(terminal),
                    None => self.notice = Some("Click the end of a pin to mark it no-connect.".into()),
                },
                Tool::PowerFlag => match self.power_flag_target(p, q, rect) {
                    Some(terminal) => self.toggle_power_flag(terminal),
                    None => {
                        self.notice = Some(
                            "Click a pin, or a wire of a net that reaches a pin, to put a power flag on it.".into(),
                        )
                    }
                },
                Tool::Power => self.place_power_symbol(q),
                Tool::Label => {
                    let name = self.label_text.trim().to_owned();
                    if name.is_empty() {
                        self.notice = Some("Type a label name first.".into());
                    } else {
                        let joined = self.label_joins("Label", &name, q);
                        self.transaction(|d| {
                            d.labels.push(Label {
                                flag: false,
                                rotation: 0,
                                terminal: None,
                                id: Uuid::new_v4(),
                                name,
                                at: q,
                            })
                        });
                        match joined {
                            Some((line, true)) => self.notice = Some(line),
                            Some((line, false)) => self.note = Some(line),
                            None => {}
                        }
                        self.label_placed = true;
                    }
                }
                Tool::Wire => {
                    // A run ends where it lands on the net: a pin, a junction, a
                    // label or a wire (KiCad's rule), so A to B then C is two
                    // wires and not one run shorting all three. Asked of the sheet
                    // BEFORE this click's segments, which would contain `q`.
                    let lands = self.wire_start.is_some() && self.lands_on_net(q);
                    if let Some(a) = self.wire_start {
                        let bend = if self.vertical_first {
                            Point::new(a.x, q.y)
                        } else {
                            Point::new(q.x, a.y)
                        };
                        self.transaction(|d| {
                            d.add_wire(a, bend);
                            d.add_wire(bend, q);
                        });
                    }
                    self.wire_start = (!lands).then_some(q);
                }
            }
        }
        painter.rect_filled(rect, 0., Color32::from_rgb(17, 23, 32));
        let spacing = grid_spacing(self.zoom);
        let origin = rect.center() + self.pan;
        let mut x = rect.left() + (origin.x - rect.left()).rem_euclid(spacing);
        while x < rect.right() {
            let mut y = rect.top() + (origin.y - rect.top()).rem_euclid(spacing);
            while y < rect.bottom() {
                painter.circle_filled(Pos2::new(x, y), 0.8, Color32::from_rgb(41, 51, 65));
                y += spacing;
            }
            x += spacing;
        }
        for w in &self.document.wires {
            let color = if self.selected == Some(Selection::Wire(w.id)) {
                ACCENT
            } else {
                WIRE
            };
            let points = w.points();
            painter.add(egui::Shape::line(
                points.iter().map(|p| self.screen(*p, rect)).collect(),
                Stroke::new(2., color),
            ));
            // A wiring diagram labels each connection on its longest segment.
            if !w.connection_id.is_empty()
                && self.zoom > 0.006
                && let Some(s) = points
                    .windows(2)
                    .max_by_key(|s| s[0].x.abs_diff(s[1].x) + s[0].y.abs_diff(s[1].y))
            {
                painter.text(
                    self.screen(s[0], rect).lerp(self.screen(s[1], rect), 0.5) + Vec2::new(0., -4.),
                    Align2::CENTER_BOTTOM,
                    &w.connection_id,
                    FontId::monospace(11.),
                    color,
                );
            }
            if self.selected == Some(Selection::Wire(w.id)) {
                for (i, s) in points.windows(2).enumerate() {
                    let center = self.screen(s[0], rect).lerp(self.screen(s[1], rect), 0.5);
                    painter.rect_filled(
                        Rect::from_center_size(
                            center,
                            Vec2::splat(if i == self.selected_segment { 8. } else { 6. }),
                        ),
                        1.,
                        ACCENT,
                    );
                }
            }
        }
        for c in &self.document.components {
            // A component whose device the assembly does not offer is drawn in the
            // warning colour, so a broken link is seen on the sheet and not only in a
            // panel the user may not have open.
            let color = match self.unlinked_device(c).is_some() {
                true => board_view::WARNING,
                false => INK,
            };
            self.paint_component(&painter, c, rect, color);
            if self.selected == Some(Selection::Component(c.id)) {
                // Every gate of the part is ringed; the one a drag or Rotate acts
                // on more strongly.
                let picked = self.selected_gate();
                for (gate, bounds) in self.drawn_gates(c, rect) {
                    let width = if picked == Some(gate) { 2. } else { 1. };
                    painter.rect_stroke(
                        bounds.expand(8.),
                        3.,
                        Stroke::new(width, ACCENT),
                        egui::StrokeKind::Outside,
                    );
                }
            }
        }
        for p in &self.document.junctions {
            painter.circle_filled(self.screen(*p, rect), 4., ACCENT);
        }
        for l in &self.document.labels {
            if self.names_power_symbol(l) {
                continue;
            }
            let tip = self.screen(l.at, rect);
            if l.flag || l.terminal.is_some() {
                let (galley, outline, text_origin) =
                    net_flag_layout(ui.ctx(), &l.name, tip, self.zoom / 0.012, l.rotation);
                let width = if self.selected == Some(Selection::Label(l.id)) {
                    2.5
                } else {
                    1.5
                };
                painter.add(egui::Shape::convex_polygon(
                    outline.to_vec(),
                    Color32::from_rgb(20, 42, 40),
                    Stroke::new(width, ACCENT),
                ));
                painter.add(
                    egui::epaint::TextShape::new(text_origin, galley, ACCENT)
                        .with_angle(l.rotation as f32 * std::f32::consts::FRAC_PI_2),
                );
            } else {
                painter.circle_filled(tip, 3., ACCENT);
                let end = self.document.terminal_at(l.at).and_then(|t| {
                    let c = self.document.components.iter().find(|c| c.id == t.component)?;
                    let pin = c.symbol.pins.iter().find(|p| p.number == t.pin)?;
                    Some(self.screen(c.pin_end(pin), rect))
                });
                let (align, offset) = label_placement(tip, end);
                painter.text(tip + offset, align, &l.name, FontId::monospace(13.), ACCENT);
            }
        }
        self.paint_erc(&painter, rect);
        if let Some(payload) = response.dnd_hover_payload::<LibraryPartDrag>()
            && let Some(p) = pointer
        {
            let at = self.snap(p, rect);
            let symbol = &payload.0.symbol;
            let preview = Component {
                id: Uuid::nil(),
                reference: payload
                    .0
                    .reference
                    .clone()
                    .unwrap_or_else(|| format!("{}?", symbol.reference_prefix)),
                value: symbol
                    .library_id
                    .split(':')
                    .next_back()
                    .unwrap_or("")
                    .into(),
                symbol: symbol.clone(),
                at,
                rotation: 0,
                part: None,
                pads: None,
                gates: vec![],
            };
            self.paint_component(&painter, &preview, rect, ACCENT);
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        }
        if let Some(p) = pointer.filter(|p| rect.contains(*p)) {
            let q = self.snap(p, rect);
            let at = self.screen(q, rect);
            if self.tool != Tool::Select {
                painter.circle_stroke(at, 5., Stroke::new(1., ACCENT));
                ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
            }
            if matches!(self.tool, Tool::Select | Tool::Wire) {
                if let Some(terminal) = self.hit_target(p, rect) {
                    let t = self.document.target_position(&terminal).unwrap();
                    let taken = self.taken(&terminal).is_some();
                    painter.circle_stroke(
                        self.screen(t, rect),
                        8.,
                        Stroke::new(2., if taken { board_view::WARNING } else { ACCENT }),
                    );
                    ui.ctx().set_cursor_icon(if taken {
                        egui::CursorIcon::NotAllowed
                    } else {
                        egui::CursorIcon::Crosshair
                    });
                } else if let Some((id, i)) = self.hit_segment(p, rect)
                    && let Some(w) = self.document.wires.iter().find(|w| w.id == id)
                {
                    let points = w.points();
                    ui.ctx().set_cursor_icon(if points[i].y == points[i + 1].y {
                        egui::CursorIcon::ResizeVertical
                    } else {
                        egui::CursorIcon::ResizeHorizontal
                    });
                }
            }
            if let Some(start) = &self.terminal_drag {
                let a = self.document.target_position(start).unwrap();
                let target = self.hit_target(p, rect).filter(|t| self.taken(t).is_none());
                let end = target
                    .as_ref()
                    .and_then(|t| self.document.target_position(t))
                    .unwrap_or(q);
                let path = target
                    .as_ref()
                    .and_then(|t| self.document.target_path(start, t))
                    .unwrap_or_else(|| vec![a, Point::new(end.x, a.y), end]);
                painter.add(egui::Shape::line(
                    path.iter().map(|p| self.screen(*p, rect)).collect(),
                    Stroke::new(2.5, ACCENT),
                ));
            }
            if self.tool == Tool::Place
                && let Some(s) = &self.placing
            {
                let c = Component {
                    id: Uuid::nil(),
                    reference: format!("{}?", s.reference_prefix),
                    value: s.library_id.split(':').next_back().unwrap_or("").into(),
                    symbol: s.clone(),
                    at: q,
                    rotation: self.rotation,
                    part: None,
                    pads: None,
                    gates: vec![],
                };
                self.paint_component(&painter, &c, rect, ACCENT);
            }
            if self.tool == Tool::Wire
                && let Some(a) = self.wire_start
            {
                let bend = if self.vertical_first {
                    Point::new(a.x, q.y)
                } else {
                    Point::new(q.x, a.y)
                };
                painter.add(egui::Shape::line(
                    vec![self.screen(a, rect), self.screen(bend, rect), at],
                    Stroke::new(2., ACCENT),
                ));
            }
            if self.tool == Tool::Label {
                painter.text(
                    at,
                    Align2::LEFT_BOTTOM,
                    &self.label_text,
                    FontId::monospace(13.),
                    ACCENT,
                );
            }
            let name = self.power_net_text.trim();
            if self.tool == Tool::Power && !name.is_empty() {
                let preview = Component {
                    id: Uuid::nil(),
                    reference: "#PWR?".into(),
                    value: name.into(),
                    symbol: power_symbol(name),
                    at: q,
                    rotation: 0,
                    part: None,
                    pads: None,
                    gates: vec![],
                };
                self.paint_component(&painter, &preview, rect, ACCENT.gamma_multiply(0.6));
            }
            self.status = format!(
                "{:?}   ·   {:.2}, {:.2} mm   ·   Grid 1.27 mm   ·   {:.0}%",
                self.tool,
                q.x as f64 / 1000.,
                q.y as f64 / 1000.,
                self.zoom / 0.012 * 100.
            );
        }
        painter.text(
            rect.left_top() + Vec2::new(20., 18.),
            Align2::LEFT_TOP,
            &self.document.title,
            FontId::proportional(14.),
            Color32::from_rgb(119, 137, 158),
        );
        if let Some(notice) = &self.notice {
            painter.text(
                rect.left_top() + Vec2::new(20., 40.),
                Align2::LEFT_TOP,
                notice,
                FontId::proportional(13.),
                board_view::WARNING,
            );
        } else if let Some(note) = &self.note {
            painter.text(
                rect.left_top() + Vec2::new(20., 40.),
                Align2::LEFT_TOP,
                note,
                FontId::proportional(13.),
                Color32::from_rgb(119, 137, 158),
            );
        }
        if self.document.components.is_empty()
            && self.document.wires.is_empty()
            && self.tool == Tool::Select
            && response.dnd_hover_payload::<LibraryPartDrag>().is_none()
        {
            // Names the pane by the name its tab carries, and says what to do there:
            // with nothing listed yet there is no part to choose, only one to add.
            let start = if self.wiring() {
                "Your wiring diagram starts here"
            } else {
                "Your circuit starts here"
            };
            let next = match (self.parts.is_empty(), &self.add_part_label) {
                (false, _) => "Choose a part from Assembly parts",
                (true, Some(_)) => "Add a part to the assembly in Assembly parts",
                (true, None) => "No parts to place yet",
            };
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                format!("{start}\n{next}"),
                FontId::proportional(20.),
                Color32::from_rgb(119, 137, 158),
            );
        }
        self.net_flag_dialog(ui.ctx());
    }
    fn net_flag_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut edit) = self.net_flag_edit.take() else {
            return;
        };
        let mut open = true;
        let mut apply = false;
        let mut cancel = false;
        egui::Window::new("Net flag")
            .collapsible(false)
            .resizable(false)
            .open(&mut open)
            .show(ctx, |ui| {
                ui.label("Same name = same electrical net, without a wire.");
                ui.label("Net name");
                let field = ui.add(
                    egui::TextEdit::singleline(&mut edit.name).hint_text("e.g. GND, +5V, SIGNAL"),
                );
                if (field.has_focus() || field.lost_focus())
                    && ui.input(|i| i.key_pressed(Key::Enter))
                    && !edit.name.trim().is_empty()
                {
                    apply = true;
                }
                if edit.focus {
                    field.request_focus();
                    edit.focus = false;
                }
                let names: std::collections::BTreeSet<_> = self
                    .document
                    .labels
                    .iter()
                    .map(|l| l.name.clone())
                    .collect();
                if !names.is_empty() {
                    ui.label("Existing nets");
                    egui::ScrollArea::vertical()
                        .max_height(150.)
                        .show(ui, |ui| {
                            for name in names {
                                if ui.selectable_label(edit.name == name, &name).clicked() {
                                    edit.name = name;
                                }
                            }
                        });
                }
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(
                            !edit.name.trim().is_empty(),
                            egui::Button::new("Apply net flag"),
                        )
                        .clicked()
                    {
                        apply = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        if apply {
            let terminal = edit.terminal.clone();
            self.transaction(|d| {
                let _ = d.set_net_flag(terminal, &edit.name);
            });
        } else if open && !cancel {
            self.net_flag_edit = Some(edit);
        }
    }
    fn paint_component(&self, painter: &egui::Painter, c: &Component, rect: Rect, color: Color32) {
        // Each gate's body, on screen: where a pin's name has room inside it.
        let mut bodies: BTreeMap<u32, Rect> = BTreeMap::new();
        for (i, g) in c.symbol.graphics.iter().enumerate() {
            let transform = |p| self.screen(c.graphic_transform(i, p), rect);
            let gate = c.symbol.graphic_gate(i);
            let mut body = |p: Pos2| {
                let r = bodies.entry(gate).or_insert(Rect::from_min_max(p, p));
                r.extend_with(p);
            };
            match g {
                Graphic::Text { at, text, size } => {
                    painter.text(
                        transform(*at),
                        Align2::CENTER_CENTER,
                        text,
                        FontId::proportional((*size as f32 * self.zoom).clamp(6., 36.)),
                        color,
                    );
                }
                Graphic::Path(points) => {
                    let points: Vec<Pos2> = points.iter().map(|p| transform(*p)).collect();
                    points.iter().for_each(|p| body(*p));
                    painter.add(egui::Shape::line(points, Stroke::new(1.8, color)));
                }
                Graphic::Circle { center, radius } => {
                    let (center, radius) = (transform(*center), *radius as f32 * self.zoom);
                    body(center - Vec2::splat(radius));
                    body(center + Vec2::splat(radius));
                    painter.circle_stroke(center, radius, Stroke::new(1.8, color));
                }
            }
        }
        // A power symbol shows its net's name past its body, away from its pin,
        // and nothing else: KiCad hides a power symbol's `#PWR` reference and its
        // one pin.
        if let Some(net) = &c.symbol.power_net {
            let pin = c.symbol.pins.first().map_or(self.screen(c.at, rect), |p| self.screen(c.pin_at(p), rect));
            let body = bodies.values().fold(Rect::from_min_max(pin, pin), |a, b| a.union(*b));
            let away = body.center() - pin;
            let (at, align) = if away.y.abs() >= away.x.abs() {
                if away.y >= 0. {
                    (Pos2::new(pin.x, body.bottom() + 2.), Align2::CENTER_TOP)
                } else {
                    (Pos2::new(pin.x, body.top() - 2.), Align2::CENTER_BOTTOM)
                }
            } else if away.x >= 0. {
                (Pos2::new(body.right() + 3., pin.y), Align2::LEFT_CENTER)
            } else {
                (Pos2::new(body.left() - 3., pin.y), Align2::RIGHT_CENTER)
            };
            painter.text(at, align, net, FontId::proportional(12.), color);
            return;
        }
        // What the pins print, by gate, so the reference is written clear of it.
        let mut printed: BTreeMap<u32, Rect> = BTreeMap::new();
        let mut print = |gate: u32, r: Rect| {
            printed.entry(gate).and_modify(|p| *p = p.union(r)).or_insert(r);
        };
        let leads: Vec<(Pos2, Pos2)> = c
            .symbol
            .pins
            .iter()
            .map(|pin| (self.screen(c.pin_at(pin), rect), self.screen(c.pin_end(pin), rect)))
            .collect();
        for (index, pin) in c.symbol.pins.iter().enumerate() {
            let (a, b) = leads[index];
            painter.line_segment([a, b], Stroke::new(1.5, color));
            let flagged = self.document.labels.iter().any(|l| {
                l.terminal
                    .as_ref()
                    .is_some_and(|t| t.component == c.id && t.pin == pin.number)
            });
            if !flagged {
                painter.circle_stroke(a, 2.5, Stroke::new(1., WIRE));
            }
            if self.zoom <= 0.006 {
                continue;
            }
            // KiCad's symbol-level `(pin_numbers (hide yes))` and `(pin_names
            // (hide yes))`, as a resistor and a connector carry them.
            let middle = a.lerp(b, 0.5);
            let across = (b.x - a.x).abs() >= (b.y - a.y).abs();
            if !c.symbol.hide_pin_numbers {
                // Over a lead that runs across, beside one that runs up or down:
                // never on the lead, and never where a net label goes (see
                // [`label_placement`]).
                let (align, offset) = if across {
                    (Align2::CENTER_BOTTOM, Vec2::new(0., -4.))
                } else {
                    (Align2::RIGHT_CENTER, Vec2::new(-4., 0.))
                };
                print(
                    pin.gate,
                    painter.text(
                        middle + offset,
                        align,
                        &pin.number,
                        FontId::monospace(10.),
                        Color32::from_rgb(145, 166, 190),
                    ),
                );
            }
            let Some(name) = pin_name(&pin.name).filter(|_| !c.symbol.hide_pin_names) else {
                continue;
            };
            // The name belongs inside the body, past the lead, as the symbol editor
            // and KiCad both show it — when it fits there. Two names that face
            // each other across a small body share it; one that would run into the
            // other ("P1P2") goes outside, under its own lead.
            let font = FontId::monospace(10.);
            let width = painter.layout_no_wrap(name.to_owned(), font.clone(), color).size().x;
            let room = bodies
                .get(&pin.gate)
                .map_or(0., |body| name_room(*body, a, b, &leads));
            let (at, align) = if width + 4. <= room {
                let (align, offset) = pin_name_placement(a, b);
                (b + offset, align)
            } else if across {
                (middle + Vec2::new(0., 3.), Align2::CENTER_TOP)
            } else {
                (middle + Vec2::new(4., 0.), Align2::LEFT_CENTER)
            };
            print(pin.gate, painter.text(at, align, name, font, color));
        }
        for (gate, bounds) in self.drawn_gates(c, rect) {
            // Beside the body, clear of what its pins print; ABOVE it when a pin
            // leaves from its right side, where that pin's wire and net label go
            // (an op-amp's output).
            let right_lead = c.symbol.pins.iter().zip(&leads).any(|(pin, (a, b))| {
                (c.gates.is_empty() || pin.gate == gate)
                    && a.x >= bounds.right() - 1.
                    && (a.x - b.x).abs() > (a.y - b.y).abs()
            });
            let clear = printed
                .get(&gate)
                .filter(|_| !c.gates.is_empty())
                .or(printed.get(&0).filter(|_| c.gates.is_empty()))
                .map_or(bounds, |p| bounds.union(*p));
            let (anchor, align) = if right_lead {
                (bounds.left_top() + Vec2::new(0., -6.), Align2::LEFT_BOTTOM)
            } else {
                (Pos2::new(clear.right(), bounds.top()) + Vec2::new(9., 0.), Align2::LEFT_TOP)
            };
            painter.text(
                anchor,
                align,
                // A part with no value, such as one from a symbol nobody has named
                // yet, shows its reference alone rather than a blank second line.
                if c.value.trim().is_empty() {
                    c.gate_reference(gate)
                } else {
                    format!("{}\n{}", c.gate_reference(gate), c.value)
                },
                FontId::proportional(12.),
                color,
            );
        }
    }
}
/// How much room, in screen points, a pin's name has inside `body` past the
/// pin's body end `b` (its terminal is `a`): up to the far side of the body,
/// or to halfway when another pin's body end faces this one across it on the
/// same line. `0` when the lead does not end on the body at all.
pub(crate) fn name_room(body: Rect, a: Pos2, b: Pos2, leads: &[(Pos2, Pos2)]) -> f32 {
    let inward = (b - a).normalized();
    if !body.expand(1.).contains(b) || !inward.is_finite() {
        return 0.;
    }
    let far = |p: Pos2| {
        let to = |edge: f32, from: f32, d: f32| if d.abs() < 0.5 { f32::INFINITY } else { (edge - from) / d };
        to(if inward.x > 0. { body.max.x } else { body.min.x }, p.x, inward.x)
            .min(to(if inward.y > 0. { body.max.y } else { body.min.y }, p.y, inward.y))
    };
    let room = far(b).max(0.);
    let facing = leads.iter().any(|(oa, ob)| {
        let theirs = (*ob - *oa).normalized();
        *ob != b
            && theirs.dot(inward) < -0.9
            && (*ob - b).dot(inward) > 0.
            && ((*ob - b) - inward * (*ob - b).dot(inward)).length() < 1.
    });
    if facing { room / 2. } else { room }
}
/// Where a free net label's name goes, when the label sits on a pin whose lead
/// runs from `tip` to `end`: on the far side of the tip from the body, so it
/// never overprints the pin's number or name. `None` when it is not on a pin.
pub(crate) fn label_placement(tip: Pos2, end: Option<Pos2>) -> (Align2, Vec2) {
    let Some(end) = end else {
        return (Align2::LEFT_BOTTOM, Vec2::new(5., -5.));
    };
    let (dx, dy) = (end.x - tip.x, end.y - tip.y);
    if dx.abs() >= dy.abs() {
        if dx > 0. {
            // The body is to the right: read leftward from the tip.
            (Align2::RIGHT_BOTTOM, Vec2::new(-5., -3.))
        } else {
            (Align2::LEFT_BOTTOM, Vec2::new(5., -3.))
        }
    } else if dy < 0. {
        // The body is above: below the tip, beside where the wire leaves.
        (Align2::LEFT_TOP, Vec2::new(5., 3.))
    } else {
        (Align2::LEFT_BOTTOM, Vec2::new(5., -5.))
    }
}
/// A pin's name, or `None` when it has none: KiCad writes an unnamed pin as `~`.
pub(crate) fn pin_name(name: &str) -> Option<&str> {
    let name = name.trim();
    (!name.is_empty() && name != "~").then_some(name)
}

/// Where a pin's name goes, given the lead from its terminal `at` to its body end
/// `b` on screen: just beyond the body end, reading away from the terminal.
pub(crate) fn pin_name_placement(at: Pos2, b: Pos2) -> (Align2, Vec2) {
    let (dx, dy) = (b.x - at.x, b.y - at.y);
    if dx.abs() >= dy.abs() {
        if dx >= 0. {
            (Align2::LEFT_CENTER, Vec2::new(4., 0.))
        } else {
            (Align2::RIGHT_CENTER, Vec2::new(-4., 0.))
        }
    } else if dy >= 0. {
        (Align2::CENTER_TOP, Vec2::new(0., 3.))
    } else {
        (Align2::CENTER_BOTTOM, Vec2::new(0., -3.))
    }
}

/// The arrow shoulder is one half-height behind the terminal, so the two
/// diagonal edges meet at a 90-degree tip. Text stays inside the rectangular body.
fn net_flag_layout(
    ctx: &egui::Context,
    name: &str,
    tip: Pos2,
    scale: f32,
    rotation: u8,
) -> (std::sync::Arc<egui::Galley>, [Pos2; 5], Pos2) {
    let font = FontId::monospace(13. * scale);
    let galley = zoomed_text(ctx, name.into(), font, ACCENT);
    let padding = 5. * scale;
    let half_height = galley.size().y / 2. + padding;
    let shoulder = tip.x - half_height;
    let left = shoulder - galley.size().x - 2. * padding;
    let outline = [
        Pos2::new(left, tip.y - half_height),
        Pos2::new(shoulder, tip.y - half_height),
        tip,
        Pos2::new(shoulder, tip.y + half_height),
        Pos2::new(left, tip.y + half_height),
    ];
    let origin = Pos2::new(left + padding, tip.y - galley.size().y / 2.);
    let rotate = egui::emath::Rot2::from_angle(rotation as f32 * std::f32::consts::FRAC_PI_2);
    (
        galley,
        outline.map(|p| tip + rotate * (p - tip)),
        tip + rotate * (origin - tip),
    )
}

/// A part's card in the library: its symbol drawn small, its name, and its label or
/// description. A part already on the sheet is drawn dimmed and says so.
fn part_card(ui: &mut egui::Ui, part: &Part, selected: bool, placed: bool) -> egui::Response {
    let symbol = &part.symbol;
    let description = symbol
        .description
        .split(", script generated")
        .next()
        .unwrap_or("");
    let name = part.name.as_str();
    let about = match (&part.reference, placed) {
        (Some(reference), true) => format!("{reference} · on the sheet"),
        (Some(reference), false) => reference.clone(),
        (None, _) => description.to_owned(),
    };
    let (rect, response) = ui.allocate_exact_size(
        Vec2::new(ui.available_width(), 88.),
        if placed { Sense::click() } else { Sense::click_and_drag() },
    );
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Button, !placed, format!("{name}\n{about}"))
    });
    let response = if placed {
        response.on_hover_text(format!("{name}\n{about}\nRight-click to open the part."))
    } else {
        response
            .on_hover_text(format!(
                "{name}\n{about}\nDrag onto the sheet, or click to place. Right-click to open the part."
            ))
            .on_hover_cursor(egui::CursorIcon::Grab)
    };
    // The theme's own widget grounds and text, light or dark. A card used to
    // be painted a hand-set navy in either, and on the light theme's panel it
    // was the one dark block in the pane.
    let visuals = ui.visuals();
    let fill = if placed {
        None
    } else if selected || response.dragged() {
        Some(visuals.selection.bg_fill)
    } else if response.hovered() {
        Some(visuals.widgets.hovered.weak_bg_fill)
    } else {
        Some(visuals.widgets.inactive.weak_bg_fill)
    };
    // A placed card is not a button any more, so it is drawn as none: an
    // outline on the pane's own ground, its words in the theme's weak colour.
    // Weak words on a widget ground, or faded ones, measured 2.69:1 on the
    // light theme, under the 2.77:1 of its own weakest text on its panel.
    let (ink, faint) = if placed {
        (visuals.weak_text_color(), visuals.weak_text_color())
    } else {
        (visuals.strong_text_color(), visuals.text_color())
    };
    let (lines, dots) = (visuals.text_color(), accent(ui));
    match fill {
        Some(fill) => {
            ui.painter().rect_filled(rect, 5., fill);
        }
        None => {
            ui.painter().rect_stroke(
                rect.shrink(0.5),
                5.,
                visuals.widgets.noninteractive.bg_stroke,
                egui::StrokeKind::Inside,
            );
        }
    }
    let painter = ui.painter().clone();
    let mut preview = painter.clone();
    if placed {
        preview.multiply_opacity(0.45);
    }
    paint_symbol_preview(
        &preview,
        symbol,
        Rect::from_min_size(rect.min + Vec2::new(8., 10.), Vec2::new(62., 68.)),
        lines,
        dots,
    );
    let width = (rect.width() - 88.).max(20.);
    let title = painter.layout(name.into(), FontId::proportional(13.), ink, width);
    painter.galley(rect.min + Vec2::new(80., 15.), title.clone(), ink);
    let subtitle = if about.chars().count() > 32 {
        format!("{}…", about.chars().take(31).collect::<String>())
    } else {
        about
    };
    let subtitle = painter.layout(subtitle, FontId::proportional(11.), faint, width);
    painter.with_clip_rect(rect.shrink(5.)).galley(
        rect.min + Vec2::new(80., 20. + title.size().y),
        subtitle,
        faint,
    );

    response
}
/// A symbol drawn small, its lines in `ink` and its pin tips in `dots`: the
/// card's theme gives both, since the card is drawn on the theme's ground.
fn paint_symbol_preview(painter: &egui::Painter, symbol: &Symbol, rect: Rect, ink: Color32, dots: Color32) {
    let mut bounds = Rect::NOTHING;
    let point = |p: Point| Pos2::new(p.x as f32, p.y as f32);
    for g in &symbol.graphics {
        match g {
            Graphic::Text { at, .. } => bounds.extend_with(point(*at)),
            Graphic::Path(points) => {
                for p in points {
                    bounds.extend_with(point(*p));
                }
            }
            Graphic::Circle { center, radius } => {
                bounds = bounds.union(Rect::from_center_size(
                    point(*center),
                    Vec2::splat(*radius as f32 * 2.),
                ))
            }
        }
    }
    for p in &symbol.pins {
        bounds.extend_with(point(p.at));
        bounds.extend_with(point(p.end));
    }
    if !bounds.is_finite() {
        return;
    }
    let scale = ((rect.width() - 8.) / bounds.width().max(1270.))
        .min((rect.height() - 8.) / bounds.height().max(1270.));
    let transform = |p: Point| rect.center() + (point(p) - bounds.center()) * scale;
    for g in &symbol.graphics {
        match g {
            Graphic::Text { at, text, size } => {
                painter.text(
                    transform(*at),
                    Align2::CENTER_CENTER,
                    text,
                    FontId::proportional((*size as f32 * scale).clamp(4., 10.)),
                    ink,
                );
            }
            Graphic::Path(points) => {
                painter.add(egui::Shape::line(
                    points.iter().map(|p| transform(*p)).collect(),
                    Stroke::new(1.5, ink),
                ));
            }
            Graphic::Circle { center, radius } => {
                painter.circle_stroke(
                    transform(*center),
                    *radius as f32 * scale,
                    Stroke::new(1.5, ink),
                );
            }
        }
    }
    for p in &symbol.pins {
        painter.line_segment([transform(p.at), transform(p.end)], Stroke::new(1.2, ink));
        painter.circle_filled(transform(p.at), 2., dots);
    }
}
fn segment_distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(0.001)).clamp(0., 1.);
    p.distance(a + ab * t)
}


