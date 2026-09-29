//! PCB layout view: footprint placement, manual routing, autorouting, and checks.
use super::*;
use crate::actions::{Action, ActionGroup, key, sheet_action};
use brep_ecad_core::autoroute::RouteJob;
use brep_ecad_core::board::{
    Airwire, Board, BoardSync, ClassReason, Connectivity, CopperRef, DEFAULT_CLASS, DesignRules, NetClass, Pad, Placement, Shape, Track,
    Via, ViaPolicy, Violation, ViolationKind, Zone, layer_name, needs_footprint, rectangle,
};
use brep_ecad_core::board::zones::{self, ZoneReport};
use crate::symbol_editor::{InspectorHits, key_word};
use brep_ecad_core::footprint;
use std::collections::{BTreeMap, BTreeSet};

/// Grid for dragging footprints and drawing tracks by hand.
const PLACE_GRID: i32 = 250;
/// Autorouter work per frame, in grid node expansions.
const ROUTE_BUDGET: u64 = 120_000;
pub(crate) const BACKGROUND: Color32 = Color32::from_rgb(14, 19, 26);
const SUBSTRATE: Color32 = Color32::from_rgb(19, 34, 33);
const EDGE: Color32 = Color32::from_rgb(222, 196, 76);
pub(crate) const THROUGH_HOLE: Color32 = Color32::from_rgb(201, 162, 72);
const VIA: Color32 = Color32::from_rgb(178, 184, 192);
pub(crate) const SILK: Color32 = Color32::from_rgb(222, 226, 232);
pub(crate) const WARNING: Color32 = Color32::from_rgb(238, 120, 96);

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum View {
    #[default]
    Schematic,
    Board,
}
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BoardTool {
    #[default]
    Select,
    Route,
    /// Draw a copper zone's outline, corner by corner.
    Zone,
}
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BoardSelection {
    Part(Uuid),
    /// One PAD of a placed part: the component it belongs to, and the pad's
    /// index in that placement's footprint. The index is the identity inside
    /// the view, because a footprint may carry several pads with no number at
    /// all (mechanical pads) and they are as selectable as the numbered ones;
    /// [`Editor::select_board_pad`] and [`Editor::selected_board_pad`] speak
    /// the NUMBER, which is the identity a pad shares with a pin and with a
    /// part's connection point.
    Pad(Uuid, usize),
    /// One SILKSCREEN line of a placed part: the component, and the line's
    /// index in that placement's footprint — the Pads editor's `silk:<index>`
    /// identity, seen on the board. Silk is legend, never copper: selecting it
    /// says which line of which part the pointer caught, and everything else it
    /// answers for its part, as a pad does.
    Silk(Uuid, usize),
    Track(Uuid),
    Via(Uuid),
    /// A copper zone, by its id.
    Zone(Uuid),
}
/// The part a board selection belongs to: a pad's is the part whose footprint
/// carries it, so everything a part answers for — rotate, flip, a drag, its own
/// right-click menu — answers for a pad of it too.
fn part_of(selection: Option<BoardSelection>) -> Option<Uuid> {
    match selection {
        Some(
            BoardSelection::Part(id) | BoardSelection::Pad(id, _) | BoardSelection::Silk(id, _),
        ) => Some(id),
        _ => None,
    }
}

fn board(e: &Editor) -> bool {
    e.view == View::Board
}
fn routing_tool(e: &Editor) -> bool {
    board(e) && e.board_view.tool == BoardTool::Route
}
fn drafting(e: &Editor) -> bool {
    e.board_view.draft.is_some()
}
fn zone_tool(e: &Editor) -> bool {
    board(e) && e.board_view.tool == BoardTool::Zone
}
/// Choosing copper layer `layer` (from 0) on a board with at least `layer + 1` layers.
const fn layer_action(
    id: &'static str,
    label: &'static str,
    run: fn(&mut Editor),
) -> Action<Editor> {
    sheet_action(id, label, ActionGroup::Layer, run)
}
fn offers_layer(e: &Editor, layer: u8) -> bool {
    board(e) && layer < e.document.board.layer_count
}
fn on_layer(e: &Editor, layer: u8) -> bool {
    e.board_view
        .active_layer
        .min(e.document.board.layer_count - 1)
        == layer
}

/// The board's actions; see [`crate::actions`].
pub(crate) static BOARD_ACTIONS: &[Action<Editor>] = &[
    sheet_action("board.tool.select", "Select", ActionGroup::Tool, |e| {
        e.set_board_tool(BoardTool::Select)
    })
    .offered_when(board)
    .pressed_when(|e| e.board_view.tool == BoardTool::Select),
    sheet_action("board.tool.route", "Route track", ActionGroup::Tool, |e| {
        e.set_board_tool(BoardTool::Route)
    })
    .keys(&[key(Key::X)])
    .offered_when(board)
    .pressed_when(|e| e.board_view.tool == BoardTool::Route),
    sheet_action("board.tool.zone", "Draw zone", ActionGroup::Tool, |e| {
        e.set_board_tool(BoardTool::Zone)
    })
    .offered_when(board)
    .pressed_when(|e| e.board_view.tool == BoardTool::Zone),
    layer_action("board.layer.1", "Copper layer 1", |e| e.choose_layer(0))
        .offered_when(|e| offers_layer(e, 0))
        .pressed_when(|e| on_layer(e, 0))
        .captioned(|e| layer_name(0, e.document.board.layer_count)),
    layer_action("board.layer.2", "Copper layer 2", |e| e.choose_layer(1))
        .offered_when(|e| offers_layer(e, 1))
        .pressed_when(|e| on_layer(e, 1))
        .captioned(|e| layer_name(1, e.document.board.layer_count)),
    layer_action("board.layer.3", "Copper layer 3", |e| e.choose_layer(2))
        .offered_when(|e| offers_layer(e, 2))
        .pressed_when(|e| on_layer(e, 2))
        .captioned(|e| layer_name(2, e.document.board.layer_count)),
    layer_action("board.layer.4", "Copper layer 4", |e| e.choose_layer(3))
        .offered_when(|e| offers_layer(e, 3))
        .pressed_when(|e| on_layer(e, 3))
        .captioned(|e| layer_name(3, e.document.board.layer_count)),
    layer_action("board.layer.5", "Copper layer 5", |e| e.choose_layer(4))
        .offered_when(|e| offers_layer(e, 4))
        .pressed_when(|e| on_layer(e, 4))
        .captioned(|e| layer_name(4, e.document.board.layer_count)),
    layer_action("board.layer.6", "Copper layer 6", |e| e.choose_layer(5))
        .offered_when(|e| offers_layer(e, 5))
        .pressed_when(|e| on_layer(e, 5))
        .captioned(|e| layer_name(5, e.document.board.layer_count)),
    sheet_action(
        "board.route.corner",
        "Corner direction",
        ActionGroup::Context,
        |e| e.board_view.diagonal_first = !e.board_view.diagonal_first,
    )
    .offered_when(routing_tool)
    .captioned(|e| {
        if e.board_view.diagonal_first {
            "Corner: 45° first"
        } else {
            "Corner: straight first"
        }
        .into()
    }),
    sheet_action("board.route.via", "Place via", ActionGroup::Context, |e| {
        let next = (e.board_view.active_layer + 1) % e.document.board.layer_count;
        e.switch_layer(next);
    })
    .keys(&[key(Key::V)])
    .offered_when(routing_tool)
    .enabled_when(|e| drafting(e) && e.document.board.layer_count > 1),
    sheet_action(
        "board.route.finish",
        "Finish track",
        ActionGroup::Context,
        Editor::finish_draft,
    )
    .keys(&[key(Key::Enter)])
    .offered_when(routing_tool)
    .enabled_when(drafting),
    sheet_action(
        "board.route.done",
        "Done routing",
        ActionGroup::Context,
        |e| {
            e.board_view.draft = None;
            e.board_view.tool = BoardTool::Select;
        },
    )
    .offered_when(routing_tool),
    sheet_action(
        "board.zone.finish",
        "Close zone",
        ActionGroup::Context,
        Editor::finish_zone,
    )
    .keys(&[key(Key::Enter)])
    .offered_when(zone_tool)
    .enabled_when(|e| e.board_view.zone_draft.len() >= 3),
    sheet_action("board.zones.fill", "Fill zones", ActionGroup::Context, |e| {
        e.fill_zones()
    })
    .keys(&[key(Key::B)])
    .offered_when(board)
    .enabled_when(|e| !e.document.board.zones.is_empty()),
    sheet_action(
        "board.rotate",
        "Rotate 90°",
        ActionGroup::Selection,
        Editor::rotate_part,
    )
    .keys(&[key(Key::R)])
    .offered_when(board)
    .enabled_when(|e| part_of(e.board_view.selected).is_some()),
    sheet_action(
        "board.delete",
        "Delete selection",
        ActionGroup::Selection,
        Editor::delete_board_selection,
    )
    .keys(&[key(Key::Delete), key(Key::Backspace)])
    .offered_when(board)
    .enabled_when(|e| e.board_view.selected.is_some()),
    sheet_action("board.cancel", "Cancel", ActionGroup::Selection, |e| {
        e.cancel_board();
        e.board_view.selected = None;
    })
    .keys(&[key(Key::Escape)])
    .offered_when(board),
];

/// A track being drawn by hand; completed layer runs and vias wait in `tracks`/`vias`.
struct Draft {
    layer: u8,
    width: i32,
    points: Vec<Point>,
    tracks: Vec<Track>,
    vias: Vec<Via>,
}

/// Where a track being drawn comes closer than the clearance between its net and
/// another's to copper that is not its own: the gap in micrometres, what that
/// copper is, the net it carries, where it is, and the clearance the two nets'
/// classes ask for with the class that asks it ([`DesignRules::clearance_between`]).
#[derive(Clone, Debug, PartialEq)]
struct Clash {
    gap: f64,
    what: String,
    net: Option<String>,
    at: Point,
    needed: i32,
    class: String,
}
impl Clash {
    /// Copper that touches is a short, not a narrow gap.
    fn crosses(&self) -> bool {
        self.gap < 0.5
    }
    /// "0.145 mm from R2 pad 2 (GND)", or "across R2 pad 2 (GND)" when it touches.
    fn words(&self) -> String {
        let net = self.net.as_deref().map_or(String::new(), |net| format!(" ({net})"));
        if self.crosses() {
            format!("across {}{net}", self.what)
        } else {
            format!("{:.3} mm from {}{net}", self.gap / 1000., self.what)
        }
    }
    /// "0.200 mm needed (Default)", as the status line and the click's message say it.
    fn needed_words(&self) -> String {
        format!("{:.3} mm needed ({})", f64::from(self.needed) / 1000., self.class)
    }
}

/// One leg a routing click would add: its points (from the draft's last point
/// to the target), the clash it still has, and the clash of the corner it
/// turned away from, if it turned.
struct Leg {
    points: Vec<Point>,
    clash: Option<Clash>,
    avoided: Option<Clash>,
}

/// What a part being dragged would come down on, judged every frame of the drag
/// as the route tool judges a draft: each of its pads that comes within the
/// board's clearance of another net's copper, closest first (a short before a
/// narrow gap), and each part on its side whose courtyard its own overlaps.
#[derive(Clone, Debug, Default, PartialEq)]
struct Landfall {
    /// The pad of the dragged part, as it is named ("C1 pad 1"), and its clash.
    copper: Vec<(String, Clash)>,
    /// The references of the parts whose courtyards it overlaps, and where each is.
    courtyards: Vec<(String, Point)>,
}
impl Landfall {
    fn is_clear(&self) -> bool {
        self.copper.is_empty() && self.courtyards.is_empty()
    }
    /// The worst of it in a phrase: "short: C1 pad 1 is across U1 pad 1 (OUT)",
    /// "clearance: C1 pad 2 is 0.120 mm from R2 pad 1 (GND)", "courtyard: C1
    /// overlaps U1", with how many more there are.
    fn words(&self, reference: &str) -> Option<String> {
        let worst = if let Some((pad, clash)) = self.copper.first() {
            let kind = if clash.crosses() { "short" } else { "clearance" };
            format!("{kind}: {pad} is {}", clash.words())
        } else {
            let (other, _) = self.courtyards.first()?;
            format!("courtyard: {reference} overlaps {other}")
        };
        Some(match self.copper.len() + self.courtyards.len() - 1 {
            0 => worst,
            n => format!("{worst}, and {n} more"),
        })
    }
}

/// What a board drag moves. A pad is never one — pressing a pad drags its part.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Subject {
    Part(Uuid),
    Via(Uuid),
    Track(Uuid, Grip),
    Zone(Uuid, ZoneGrip),
}

/// What a press on a zone takes hold of: one corner, the middle of one edge (a
/// drag there puts a new corner in), or the whole zone.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ZoneGrip {
    Corner(usize),
    Edge(usize),
    Body,
}

/// The middle of each edge of a zone's outline, edge `i` running from corner `i`.
fn edge_middles(outline: &[Point]) -> Vec<Point> {
    let n = outline.len();
    (0..n)
        .map(|i| {
            let (a, b) = (outline[i], outline[(i + 1) % n]);
            Point::new(
                ((i64::from(a.x) + i64::from(b.x)) / 2) as i32,
                ((i64::from(a.y) + i64::from(b.y)) / 2) as i32,
            )
        })
        .collect()
}

/// A zone's outline after a drag of `grip` by `delta`, from the outline it had.
fn dragged_outline(outline: &[Point], grip: ZoneGrip, delta: Point) -> Vec<Point> {
    let mut out = outline.to_vec();
    match grip {
        ZoneGrip::Corner(i) => {
            if let Some(p) = out.get_mut(i) {
                *p = p.offset(delta);
            }
        }
        ZoneGrip::Edge(i) => {
            if let Some(&middle) = edge_middles(outline).get(i) {
                out.insert(i + 1, middle.offset(delta));
            }
        }
        ZoneGrip::Body => {
            for p in &mut out {
                *p = p.offset(delta);
            }
        }
    }
    out
}

/// The board with every zone's fill left out: what a fill is made FROM, compared
/// to say whether the fill on show is still the one the copper asks for.
fn fill_inputs_match(a: &Board, b: &Board) -> bool {
    a.outline == b.outline
        && a.layer_count == b.layer_count
        && a.placements == b.placements
        && a.tracks == b.tracks
        && a.vias == b.vias
        && a.rules == b.rules
        && a.zones.len() == b.zones.len()
        && a.zones.iter().zip(&b.zones).all(|(x, y)| {
            x.id == y.id
                && x.net == y.net
                && x.layer == y.layer
                && x.outline == y.outline
                && x.thermal_gap == y.thermal_gap
                && x.spoke_width == y.spoke_width
                && x.min_width == y.min_width
        })
}

/// What the canvas says of a stale fill: zones never filled are "not filled",
/// a fill that no longer matches the copper is "out of date".
fn stale_words(board: &Board) -> &'static str {
    if board.zones.iter().all(|z| z.fill.is_empty()) {
        "zones not filled · B fills them"
    } else {
        "fill out of date · B refills"
    }
}

/// A zone fill's report in one line: "GND on B.Cu: 1107.8 mm² in 1 piece; J1.1 3
/// spokes, J2.1 3 spokes".
fn zone_words(zone: &Zone, report: &ZoneReport, layer_count: u8, pads: &dyn Fn(CopperRef) -> String) -> String {
    let head = zones::describe(zone, layer_count);
    // A netless via this zone left alone because another net's zone holds it
    // too: the fill keeps clear of it rather than short the two nets through it.
    let unstitched: String = report
        .unstitched
        .iter()
        .map(|(at, others)| {
            format!(
                " The via at {:.2}, {:.2} mm has no net and a {} zone holds it too, so it is stitched to neither.",
                mm(at.x),
                mm(at.y),
                others.join(" and ")
            )
        })
        .collect();
    if let Some(why) = &report.empty_because {
        return format!("{head}: nothing filled, because {why}.{unstitched}");
    }
    let pieces = match report.pieces {
        1 => "1 piece".to_owned(),
        n => format!("{n} pieces"),
    };
    let mut words = format!("{head}: {:.1} mm² in {pieces}", report.area / 1e6);
    if report.islands_removed > 0 {
        words += &format!(", {} unconnected island(s) removed", report.islands_removed);
    }
    let thermals: Vec<String> = report
        .thermals
        .iter()
        .map(|t| match t.spokes {
            0 => format!("{} not reached", pads(t.owner)),
            1 => format!("{} 1 spoke", pads(t.owner)),
            n => format!("{} {n} spokes", pads(t.owner)),
        })
        .collect();
    if !thermals.is_empty() {
        words += &format!("; thermal reliefs: {}", thermals.join(", "));
    }
    words + "." + &unstitched
}

/// What a routing click landed on.
#[derive(Clone, Copy, PartialEq)]
enum Anchor {
    Pad { layers: (u8, u8), width: i32 },
    Via,
    Track { layer: u8, width: i32 },
}

pub(crate) struct BoardView {
    pub tool: BoardTool,
    pub selected: Option<BoardSelection>,
    pan: Vec2,
    zoom: f32,
    fitted: bool,
    pub active_layer: u8,
    hidden: BTreeSet<u8>,
    show_ratsnest: bool,
    /// A move in progress: the document it started from, where the pointer pressed,
    /// where the dragged thing was, and which thing it is.
    drag: Option<(Document, Point, Point, Subject)>,
    /// The pad a dragged free track end is over this frame, if any: drawn as a
    /// ring while the drag lasts, and named on release when it was refused.
    landing: Option<Landing>,
    /// While a PART is dragged: the copper of the board as it was when the drag
    /// began, whose islands name the net each piece of other copper is on, and
    /// what the part would come down on where it is now ([`Editor::landfall`]).
    placing: Option<(Connectivity, Landfall)>,
    draft: Option<Draft>,
    diagonal_first: bool,
    job: Option<RouteJob>,
    pub message: String,
    pub(crate) violations: Option<Vec<Violation>>,
    /// The finding last clicked in the list, by its index in [`Self::violations`]:
    /// its row stays selected and its mark on the board is drawn larger.
    focused_violation: Option<usize>,
    rules: Option<DesignRules>,
    outline: Option<(f32, f32)>,
    net_width: (String, f32),
    /// The net the Net classes section reads out and assigns when nothing on the
    /// board with a net is selected, chosen from its own picker.
    class_net: String,
    /// A net class name or pattern list being typed that is not (yet) valid, by
    /// the class's place in the list and the field; the model keeps the last
    /// valid one, and the Inspector says why this one is not.
    class_text: BTreeMap<(usize, &'static str), (String, String)>,
    unrouted: usize,
    canvas: Rect,
    /// The Inspector's widgets, `inspector:<thing>`, where [`Editor::board_inspector`]
    /// last drew them, published with the canvas's hits ([`Editor::board_hits`]).
    inspector: InspectorHits,
    /// The canvas's right-click menu's entries, `menuitem:<entry>`, while it is open.
    menu: Vec<(String, Rect)>,
    /// The pass the canvas was last drawn in.
    canvas_pass: u64,
    /// The corners of a zone being drawn with the Zone tool.
    zone_draft: Vec<Point>,
    /// The net a zone drawn next pours: chosen in the Inspector, GND until then.
    pub(crate) zone_net: String,
    /// What each zone's last fill in this session did, by zone.
    zone_reports: BTreeMap<Uuid, ZoneReport>,
    /// The board the zones were last filled from, fills left out of the
    /// comparison ([`fill_inputs_match`]): a board that no longer matches it is
    /// showing a stale fill. Taken from the first board seen when nothing has
    /// been filled yet ([`Editor::seed_fill_baseline`]).
    filled_from: Option<Board>,
    /// Whether the fill the document was OPENED with already differed from what
    /// a fill of its copper makes — saved stale, a track drawn after the pour —
    /// which a comparison with the board as opened cannot see. Cleared by the
    /// next fill.
    opened_stale: bool,
}
impl Default for BoardView {
    fn default() -> Self {
        Self {
            tool: BoardTool::Select,
            selected: None,
            pan: Vec2::ZERO,
            zoom: 0.012,
            fitted: false,
            active_layer: 0,
            hidden: BTreeSet::new(),
            show_ratsnest: true,
            drag: None,
            landing: None,
            placing: None,
            draft: None,
            diagonal_first: false,
            job: None,
            message: String::new(),
            violations: None,
            focused_violation: None,
            rules: None,
            outline: None,
            net_width: (String::new(), 0.4),
            class_net: String::new(),
            class_text: BTreeMap::new(),
            unrouted: 0,
            canvas: Rect::from_min_size(Pos2::ZERO, Vec2::new(800., 600.)),
            inspector: InspectorHits::default(),
            menu: vec![],
            canvas_pass: 0,
            zone_draft: vec![],
            zone_net: String::new(),
            zone_reports: BTreeMap::new(),
            filled_from: None,
            opened_stale: false,
        }
    }
}
impl BoardView {
    /// The board's pan and zoom, as [`crate::Editor::pan_zoom`] reports them.
    pub(crate) fn pan_zoom(&self) -> (Vec2, f32) {
        (self.pan, self.zoom)
    }
    fn screen(&self, p: Point) -> Pos2 {
        self.canvas.center() + self.pan + Vec2::new(p.x as f32, p.y as f32) * self.zoom
    }
    fn world(&self, p: Pos2) -> Point {
        let v = (p - self.canvas.center() - self.pan) / self.zoom;
        Point::new(v.x.round() as i32, v.y.round() as i32)
    }
    fn px(&self, micrometres: i32) -> f32 {
        micrometres as f32 * self.zoom
    }
    /// Pointer tolerance in micrometres.
    fn tolerance(&self) -> f64 {
        f64::from(6. / self.zoom)
    }
    fn visible(&self, layer: u8) -> bool {
        !self.hidden.contains(&layer)
    }
    fn fit(&mut self, board: &Board) {
        let (min, max) = board.outline_bounds();
        let size = Vec2::new(
            (max.x - min.x).max(1000) as f32,
            (max.y - min.y).max(1000) as f32,
        );
        let available = self.canvas.size() - Vec2::splat(60.);
        self.zoom = crate::valid_zoom(
            (available.x / size.x).min(available.y / size.y).min(0.2),
            self.zoom,
        );
        self.pan = -Vec2::new((min.x + max.x) as f32 / 2., (min.y + max.y) as f32 / 2.) * self.zoom;
        self.fitted = true;
    }
    fn zoom_about(&mut self, factor: f32, anchor: Pos2) {
        crate::zoom_at(&mut self.zoom, &mut self.pan, factor, anchor - self.canvas.center());
    }
    fn centre_on(&mut self, p: Point) {
        self.pan = -Vec2::new(p.x as f32, p.y as f32) * self.zoom;
    }
    /// Centre on a finding at `p` that spans `extent` µm (0 for one at a point),
    /// zoomed in far enough that [`FOCUS_SPAN`] fills the canvas's shorter side —
    /// near enough to see the two things a finding is between — and never zoomed
    /// out for that; but out as far as it takes to show the whole of a finding that
    /// is longer, such as an unrouted connection's two pads, with 2 mm to spare.
    fn focus_on(&mut self, p: Point, extent: i32) {
        let side = self.canvas.size().min_elem();
        let near = crate::valid_zoom(side / FOCUS_SPAN, self.zoom);
        let whole = crate::valid_zoom(side / (extent as f32 + 4000.), self.zoom);
        self.zoom = self.zoom.max(near).min(whole);
        self.centre_on(p);
    }
}

/// How much board, in µm, a click on a design rule finding shows across the canvas's
/// shorter side ([`BoardView::focus_on`]): 8 mm, a pad pitch or two either side.
const FOCUS_SPAN: f32 = 8000.;

/// A design rule finding's row in the list: in body text, the kind in its colour and
/// the message in the text colour. The list is read line by line, and it was 9 pt
/// red (the eCAD workflow audit's item 16).
fn finding_row(style: &egui::Style, v: &Violation) -> egui::text::LayoutJob {
    let body = egui::TextStyle::Body.resolve(style);
    let color = if matches!(v.kind, ViolationKind::Unrouted | ViolationKind::Stitching) {
        style.visuals.warn_fg_color
    } else {
        style.visuals.error_fg_color
    };
    let mut row = egui::text::LayoutJob::default();
    row.append(violation_kind(v.kind), 0., egui::TextFormat::simple(body.clone(), color));
    row.append(&v.message, 8., egui::TextFormat::simple(body, style.visuals.text_color()));
    row
}

/// The word a design rule finding's kind is listed under.
fn violation_kind(kind: ViolationKind) -> &'static str {
    match kind {
        ViolationKind::Short => "Short",
        ViolationKind::Clearance => "Clearance",
        ViolationKind::TrackWidth => "Width",
        ViolationKind::BoardEdge => "Edge",
        ViolationKind::MissingPad => "Pad",
        ViolationKind::Courtyard => "Courtyard",
        ViolationKind::Stitching => "Stitching",
        ViolationKind::Unrouted => "Unrouted",
    }
}

/// The margin, in µm, that the Fit outline to parts button leaves round the
/// courtyards.
const FIT_MARGIN: i32 = 3000;

/// A line for the outline section while the board is still the stock 60 × 40 mm
/// rectangle every new board starts with and parts are on it: how much of it they
/// use, beside the button that fits it. `None` for an empty board and for an
/// outline anyone has changed.
///
/// A line rather than a fit of its own: a board that fits itself on the first
/// placement is too tight for the layout that follows — parts dragged apart land off
/// it — so the fit stays the user's to ask for once the parts are where they go.
fn stock_board_note(board: &Board) -> Option<String> {
    if board.outline != Board::default().outline {
        return None;
    }
    let (min, max) = board
        .placements
        .iter()
        .map(Placement::courtyard)
        .reduce(|(a, b), (c, d)| {
            (Point::new(a.x.min(c.x), a.y.min(c.y)), Point::new(b.x.max(d.x), b.y.max(d.y)))
        })?;
    Some(format!(
        "The stock 60 × 40 mm board. Its parts span {:.1} × {:.1} mm: Fit outline to parts sizes it round them.",
        mm(max.x - min.x),
        mm(max.y - min.y)
    ))
}

/// A board size as it is said: `40 × 21`, in millimetres, to 0.1 mm.
fn size_words(size: Point) -> String {
    let words = |v: i32| {
        let text = format!("{:.1}", mm(v));
        text.strip_suffix(".0").map_or(text.clone(), str::to_owned)
    };
    format!("{} × {}", words(size.x), words(size.y))
}

fn snap(p: Point, step: i32) -> Point {
    let round = |v: i32| ((f64::from(v) / f64::from(step)).round() * f64::from(step)) as i32;
    Point::new(round(p.x), round(p.y))
}
pub(crate) fn mm(micrometres: i32) -> f32 {
    micrometres as f32 / 1000.
}
pub(crate) fn um(millimetres: f32) -> i32 {
    (millimetres * 1000.).round() as i32
}
/// Font height for a pad's number, or `None` while the pad is too small on screen to
/// hold one. The number grows with the pad, as it does in the footprint editor.
fn pad_number_height(size: Point, zoom: f32) -> Option<f32> {
    let smallest = size.x.min(size.y) as f32 * zoom;
    (smallest >= 6.).then(|| (smallest * 0.75).clamp(6., 14.))
}
/// How a pad is named in a sentence: `Pad R1.2`, or a mechanical pad, which has
/// no number to be named by.
fn pad_title(reference: &str, pad: &Pad) -> String {
    if pad.number.is_empty() {
        format!("A mechanical pad of {reference}")
    } else {
        format!("Pad {reference}.{}", pad.number)
    }
}
fn via_policy_name(policy: ViaPolicy) -> &'static str {
    match policy {
        ViaPolicy::Allow => "Allow",
        ViaPolicy::Avoid => "Avoid when possible",
        ViaPolicy::Never => "Never",
    }
}
pub(crate) fn layer_color(layer: u8, count: u8) -> Color32 {
    if layer == 0 {
        Color32::from_rgb(214, 64, 64)
    } else if layer + 1 >= count {
        Color32::from_rgb(64, 118, 226)
    } else {
        [
            Color32::from_rgb(204, 190, 62),
            Color32::from_rgb(196, 82, 196),
            Color32::from_rgb(70, 196, 190),
            Color32::from_rgb(132, 202, 92),
        ][usize::from(layer - 1) % 4]
    }
}
/// Move the via `id` by `delta`, taking with it every track end that lands on it.
///
/// A via exists to carry one net between layers, so the copper that meets it has to
/// arrive at wherever it goes; a via that moved alone would leave its own connection
/// broken. An end counts as meeting the via when its end cap touches or overlaps the
/// via's land, which is [`Board::connectivity`]'s own join test with its 0.5 µm of
/// slack dropped: the ends that follow are the ends the board calls connected, give or
/// take that half micrometre, and routing snaps a click on a via to its centre exactly
/// so the difference is not reachable by hand.
///
/// Only the two ENDS of a track follow. A track that crosses the via has no vertex
/// there for `simplify_path` to have kept, and bending it at the crossing would deform
/// a run the user never touched; that crossing shows up as an airwire and, if it is a
/// different net, as a clearance finding.
fn move_via(board: &mut Board, id: Uuid, delta: Point) {
    let Some(land) = board
        .vias
        .iter()
        .find(|v| v.id == id)
        .map(|v| Shape::circle(v.at, v.diameter / 2))
    else {
        return;
    };
    for track in &mut board.tracks {
        if track.points.is_empty() {
            continue;
        }
        let last = track.points.len() - 1;
        for i in [Some(0), (last > 0).then_some(last)].into_iter().flatten() {
            if rests_on(&land, track.points[i], track.width) {
                track.points[i] = track.points[i].offset(delta);
            }
        }
    }
    if let Some(via) = board.vias.iter_mut().find(|v| v.id == id) {
        via.at = via.at.offset(delta);
    }
}

/// Whether a track end at `end`, `width` wide, rests on `copper`: its end cap and
/// the copper touch or overlap. This is the one land rule — [`move_via`] asks it of a
/// via's land, and a track drag asks it of every pad, via and other track — and it is
/// [`Board::connectivity`]'s join test with the 0.5 µm of slack dropped, so it never
/// holds an end the board does not already call connected.
fn rests_on(copper: &Shape, end: Point, width: i32) -> bool {
    // The cap's own radius is the reach: the copper and the cap touch at that gap.
    copper.distance_to_point(end) <= f64::from(width / 2)
}

/// Whether the end of `track` at `end` is HELD: it rests on a pad on the track's
/// layer, on a via, or on another track of the same layer. A held end is where the
/// run is connected to something, and no track gesture moves it.
fn held(board: &Board, track: &Track, end: Point) -> bool {
    let count = board.layer_count;
    let width = track.width;
    board
        .vias
        .iter()
        .any(|v| rests_on(&Shape::circle(v.at, v.diameter / 2), end, width))
        || board.placements.iter().any(|p| {
            p.footprint.pads.iter().any(|pad| {
                let layers = p.pad_layers(pad, count);
                (layers.0..=layers.1).contains(&track.layer)
                    && rests_on(&p.pad_shape(pad), end, width)
            })
        })
        || board
            .tracks
            .iter()
            .filter(|t| t.id != track.id && t.layer == track.layer)
            .any(|t| t.segments().any(|s| rests_on(&s, end, width)))
}

/// What a press grabs of a track: one corner, or one straight segment.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Grip {
    /// Point `i` of the track: an interior corner, or an end that is not held.
    Vertex(usize),
    /// The segment from point `k` to point `k + 1`.
    Segment(usize),
}

/// What a press at `q` grabs of `track`. A corner wins within reach of it — the
/// pointer tolerance, or the track's own half width when that is larger, so a fat
/// track's corner is grabbed anywhere on its rounded join. A HELD end is never a
/// corner to grab: a press there takes the segment it ends, which slides without
/// letting go of it.
fn grip(board: &Board, track: &Track, q: Point, tolerance: f64) -> Option<Grip> {
    let points = &track.points;
    let last = points.len().checked_sub(1)?;
    let reach = tolerance.max(f64::from(track.width / 2));
    let gap = |p: Point| f64::from(p.x - q.x).hypot(f64::from(p.y - q.y));
    let corner = (0..=last)
        .filter(|&i| (i != 0 && i != last) || !held(board, track, points[i]))
        .map(|i| (i, gap(points[i])))
        .filter(|(_, d)| *d <= reach)
        .min_by(|a, b| a.1.total_cmp(&b.1));
    if let Some((i, _)) = corner {
        return Some(Grip::Vertex(i));
    }
    track
        .segments()
        .enumerate()
        .map(|(k, s)| (k, s.distance_to_point(q)))
        .filter(|(_, d)| *d <= tolerance)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(k, _)| Grip::Segment(k))
}

/// Where the line through `p` along `e` crosses the line through `q` along `f`, as
/// the parameter along `e` (1 is at `p + e`) and the point; `None` when the lines
/// meet at less than about 15°. A shallow crossing is not a corner to keep: a slide
/// of `s` moves it `s / sin θ` along the neighbour — a 1 mm slide beside a 1°
/// neighbour, which an imported arc's facets are, would throw the corner 57 mm.
fn meet(p: Point, e: (f64, f64), q: Point, f: (f64, f64)) -> Option<(f64, Point)> {
    let cross = e.0 * f.1 - e.1 * f.0;
    let lengths = e.0.hypot(e.1) * f.0.hypot(f.1);
    if lengths == 0. || (cross / lengths).abs() < 0.25 {
        return None;
    }
    let (wx, wy) = (f64::from(q.x - p.x), f64::from(q.y - p.y));
    let u = (wx * f.1 - wy * f.0) / cross;
    let at = Point::new(
        (f64::from(p.x) + u * e.0).round() as i32,
        (f64::from(p.y) + u * e.1).round() as i32,
    );
    Some((u, at))
}

/// Edit one track in place and tidy it: a gesture may fold a corner flat or onto its
/// neighbour, and a zero-length segment is not a valid track.
fn edit_track(board: &mut Board, id: Uuid, edit: impl FnOnce(&Board, &[Point]) -> Vec<Point>) {
    let Some(i) = board.tracks.iter().position(|t| t.id == id) else {
        return;
    };
    let points = edit(board, &board.tracks[i].points);
    let points = brep_ecad_core::board::simplify_path(points);
    if points.len() >= 2 {
        board.tracks[i].points = points;
    }
}

/// Slide segment `k` of track `id` sideways by `delta`'s component across it.
///
/// The segment keeps its direction and its neighbours keep theirs: each end of it
/// moves along the neighbouring segment's own line to wherever that line meets the
/// moved one, so a run of 90° and 45° corners stays a run of 90° and 45° corners.
/// Where a neighbour would have to turn back on itself to do so, that end moves
/// straight across instead and the neighbour rubber-bands.
///
/// An end of the TRACK is different. If it is held — on a pad, a via, or another
/// track — it stays exactly where it is and a new corner is added: a 45° leg from
/// the held end to the moved segment, or a square one when the segment is too short
/// to take the 45°. A free end simply travels with the segment.
fn slide_segment(board: &mut Board, id: Uuid, k: usize, delta: Point) {
    edit_track(board, id, |board, points| {
        let Some(track) = board.tracks.iter().find(|t| t.id == id) else {
            return points.to_vec();
        };
        if k + 1 >= points.len() {
            return points.to_vec();
        }
        let last = points.len() - 1;
        let (a, b) = (points[k], points[k + 1]);
        let (dx, dy) = (f64::from(b.x - a.x), f64::from(b.y - a.y));
        let length = dx.hypot(dy);
        if length == 0. {
            return points.to_vec();
        }
        let d = (dx / length, dy / length);
        let n = (-d.1, d.0);
        let s = f64::from(delta.x) * n.0 + f64::from(delta.y) * n.1;
        let m = Point::new((s * n.0).round() as i32, (s * n.1).round() as i32);
        if m == Point::new(0, 0) {
            return points.to_vec();
        }
        let hold_a = k == 0 && held(board, track, a);
        let hold_b = k + 1 == last && held(board, track, b);
        let along = |p: Point, by: f64| {
            Point::new(
                (f64::from(p.x) + by * d.0).round() as i32,
                (f64::from(p.y) + by * d.1).round() as i32,
            )
        };
        let dir = |from: Point, to: Point| (f64::from(to.x - from.x), f64::from(to.y - from.y));
        let forward =
            |p: Point, q: Point| f64::from(q.x - p.x) * d.0 + f64::from(q.y - p.y) * d.1 > 0.;
        let (a_moved, b_moved) = (a.offset(m), b.offset(m));
        // Each end of the segment, keeping its neighbour's direction where it can.
        let mut start = if hold_a {
            along(a_moved, s.abs())
        } else if k > 0 {
            meet(points[k - 1], dir(points[k - 1], a), a_moved, d)
                .filter(|(u, _)| *u > 0.)
                .map_or(a_moved, |(_, at)| at)
        } else {
            a_moved
        };
        let mut end = if hold_b {
            along(b_moved, -s.abs())
        } else if k + 1 < last {
            meet(points[k + 2], dir(points[k + 2], b), a_moved, d)
                .filter(|(u, _)| *u > 0.)
                .map_or(b_moved, |(_, at)| at)
        } else {
            b_moved
        };
        // A segment that would come out reversed or folded to nothing is moved
        // square across instead, which keeps its own length.
        if !forward(start, end) {
            (start, end) = (a_moved, b_moved);
        }
        let mut out = points[..k].to_vec();
        if hold_a {
            out.push(a);
        }
        out.extend([start, end]);
        if hold_b {
            out.push(b);
        }
        out.extend_from_slice(&points[k + 2..]);
        out
    });
}

/// Move point `i` of track `id` by `delta`; the two segments meeting it rubber-band.
/// Every other track on the same layer whose END rests on that corner comes along, as
/// a track's ends come along with a via: a branch joined at a corner stays joined.
fn move_vertex(board: &mut Board, id: Uuid, i: usize, delta: Point) {
    let Some(track) = board.tracks.iter().find(|t| t.id == id) else {
        return;
    };
    let Some(&corner) = track.points.get(i) else {
        return;
    };
    let (layer, cap) = (track.layer, Shape::circle(corner, track.width / 2));
    for other in board
        .tracks
        .iter_mut()
        .filter(|t| t.id != id && t.layer == layer && !t.points.is_empty())
    {
        let last = other.points.len() - 1;
        let mut points = other.points.clone();
        for j in [Some(0), (last > 0).then_some(last)].into_iter().flatten() {
            if rests_on(&cap, points[j], other.width) {
                points[j] = points[j].offset(delta);
            }
        }
        let points = brep_ecad_core::board::simplify_path(points);
        if points.len() >= 2 {
            other.points = points;
        }
    }
    edit_track(board, id, |_, points| {
        let mut points = points.to_vec();
        points[i] = points[i].offset(delta);
        points
    });
}

/// Where a dragged free END of a track comes down on a pad: which pad, the
/// offset that puts the end on that pad's centre, and whether it may go there.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Landing {
    component: Uuid,
    index: usize,
    /// From the end's place before the drag to the pad's centre.
    delta: Point,
    /// The pad is on the track's net, or the track is on no net yet: the end
    /// snaps to its centre. `false` is a pad of ANOTHER net (or of none, for a
    /// track that has one): the end is not snapped, it keeps its grid step,
    /// and whatever it touches there is a short the design rule check reports.
    joins: bool,
}

/// The pad a dragged free end of track `id` is over, when point `i` is an end:
/// `moved` is where the pointer has taken the end, BEFORE the grid — a pad
/// smaller than a grid step is still caught when the pointer is on it. Only a
/// pad on the track's layer counts, the topmost part's first, as `hit_board`
/// picks pads. The track's net is read from `document` as it was before the
/// drag, where the free end is on nothing and so says nothing about it.
fn land_end(document: &Document, id: Uuid, i: usize, moved: Point) -> Option<Landing> {
    let board = &document.board;
    let track = board.tracks.iter().find(|t| t.id == id)?;
    let last = track.points.len().checked_sub(1)?;
    if i != 0 && i != last {
        return None;
    }
    let end = track.points[i];
    let (placement, index, pad) = board.placements.iter().rev().find_map(|placement| {
        placement
            .footprint
            .pads
            .iter()
            .enumerate()
            .find(|(_, pad)| {
                let layers = placement.pad_layers(pad, board.layer_count);
                (layers.0..=layers.1).contains(&track.layer)
                    && placement.pad_shape(pad).contains(moved)
            })
            .map(|(index, pad)| (placement, index, pad))
    })?;
    let centre = placement.transform(pad.at);
    let netlist = document.netlist();
    let pad_net = brep_ecad_core::board::pin_nets(&netlist)
        .get(&(placement.component, pad.number.clone()))
        .cloned()
        .filter(|_| !pad.number.is_empty());
    let joins = match (document.track_nets().get(&id), pad_net) {
        (None, _) => true,
        (Some(net), Some(pad_net)) => *net == pad_net,
        (Some(_), None) => false,
    };
    Some(Landing {
        component: placement.component,
        index,
        delta: Point::new(centre.x - end.x, centre.y - end.y),
        joins,
    })
}

/// Two segments from `a` to `b`: one straight and one at 45°.
fn corner(a: Point, b: Point, diagonal_first: bool) -> Vec<Point> {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let d = dx.abs().min(dy.abs());
    let bend = if diagonal_first {
        Point::new(a.x + d * dx.signum(), a.y + d * dy.signum())
    } else {
        Point::new(b.x - d * dx.signum(), b.y - d * dy.signum())
    };
    vec![a, bend, b]
}

impl Editor {
    pub fn set_view(&mut self, view: View) {
        // A wiring diagram has no nets, so it has no board.
        if self.view == view || view == View::Board && self.wiring() {
            return;
        }
        self.cancel();
        self.view = view;
        if view == View::Board {
            self.sync_board();
        }
    }
    /// Bring the board in line with the parts and the sheet, the FORCED way: every
    /// pads-bearing device the host offers is seated whether or not the offer moved,
    /// and the whole standing state is restated, including the components that have no
    /// pads and were named once already. What the "Update from parts" button does.
    pub fn sync_board(&mut self) {
        self.reported_without_pads.clear();
        let (sync, refused) = self.follow(true);
        self.announce(&sync, &refused);
        self.announce_without_pads();
    }
    /// Bring the board in line with the parts and the sheet, automatically, whenever
    /// either could have moved. Called at the top of every frame
    /// ([`Editor::show`]) and after every edit ([`Editor::commit`], which runs the
    /// second gate alone).
    ///
    /// # The two gates, and why the sweep is behind them
    ///
    /// [`brep_ecad_core::Document::sync_board`] walks every component AND every
    /// placement, looks each component up in the placements (which is quadratic in a
    /// dense board), and repacks whatever it adds. It is not a thing to run sixty
    /// times a second, so a frame on which nothing moved pays only the two gates:
    ///
    /// - [`Editor::offered_mark`], one hash over the host's `parts`, which moves when
    ///   the ASSEMBLY gained, lost or re-versioned a device. No edit in this editor
    ///   can move it, so an undo cannot make this gate fire and put back what the undo
    ///   took away;
    /// - [`Editor::board_is_stale`], two id sets over the components and the
    ///   placements. No track, no via, no courtyard and no netlist.
    ///
    /// A pass of its own is one undo step, and correctly not coalesced with anything:
    /// the edit it follows is the host's Add Component, which is in the host's history
    /// and not in this one.
    pub(crate) fn follow_parts(&mut self) {
        if self.wiring() {
            return;
        }
        let mark = self.offered_mark();
        let offer_moved = self.followed_parts != Some(mark);
        self.followed_parts = Some(mark);
        // A state already healed once is not healed again, and that is what keeps this
        // out of BREP's undo. Undo in the app is `pull` → `set_document` → `show`, not a
        // key this editor sees, so an undo that lands on a stale board arrives here as a
        // stale board — and a heal that ran again would become a fresh undo entry, and
        // the user could never step past the edit that made it stale. So the pass
        // remembers the rosters it healed FROM and lets that state alone. The board then
        // stays stale until the next edit, which heals it inside its own step
        // ([`Editor::commit`]) and is undoable with it.
        let stale = self.stale_mark().filter(|m| self.healed_stale != Some(*m));
        if offer_moved || stale.is_some() {
            if stale.is_some() {
                self.healed_stale = stale;
            }
            let (sync, refused) = self.follow(offer_moved);
            self.announce(&sync, &refused);
        }
        self.announce_without_pads();
        // Here and not only in the board view, so a board opened on its sheet
        // already says its fill is stale.
        self.seed_fill_baseline();
    }
    /// One pass: seat the offered devices the sheet has not got, if asked, then let the
    /// board follow the sheet. Returns what the sweep did and the parts it refused.
    fn follow(&mut self, seat: bool) -> (BoardSync, Vec<String>) {
        let mut sync = BoardSync::default();
        let mut refused = vec![];
        let parts = if seat { self.parts.clone() } else { vec![] };
        self.transaction(|d| {
            for part in parts.into_iter().filter(|p| p.pads.is_some()) {
                // A part with neither an occurrence nor a label cannot be told from the
                // copy already on the sheet, so it is not seated automatically: every
                // pass would put down another one.
                if part.source.instance.is_none() && part.reference.is_none() {
                    continue;
                }
                // By occurrence where BOTH sides have one; else by label. A
                // component placed before occurrences existed has none, and
                // comparing it with the offered one (Some against None) called
                // every such part missing and tried to seat it again under the
                // label it already has: "J1 is already used by another part",
                // on opening the committed ecad-assembly (the eCAD third
                // audit's B9).
                let present = d.components.iter().any(|c| {
                    match (c.part.as_ref().and_then(|s| s.instance.as_ref()), &part.source.instance) {
                        (Some(have), Some(offered)) => have == offered,
                        _ => part.reference.as_ref() == Some(&c.reference),
                    }
                });
                // Seated where the sheet has room for it, as a part the host seats
                // with no pointer is (`Document::open_spot`): parts all put at the
                // origin covered one another, and only the one drawn last could be
                // clicked. Each part seated here is on the sheet before the next
                // asks, so a batch spreads out too.
                if present {
                    continue;
                }
                let at = d.open_spot(&part.symbol);
                if let Err(refusal) =
                    d.place_part(part.source, part.symbol, part.pads, part.reference, at, 0)
                {
                    refused.push(refusal);
                }
            }
            sync = d.sync_board();
        });
        (sync, refused)
    }
    /// Whether the host's offer of devices has moved since the last pass: one hash over
    /// what a pass reads of each part — which device it is, the version it carries, the
    /// label it takes and whether it brings pads. Not the symbol or the footprint: a
    /// device whose geometry changed under the same version is the part refresh's
    /// business, and hashing it would put every pad of every device on every frame.
    ///
    /// A hash, not the list, because the list is compared sixty times a second and
    /// holding a copy of it would hold a copy of every symbol. Two offers that collide
    /// in 64 bits leave the board waiting for the next change or for the button.
    fn offered_mark(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::hash::DefaultHasher::new();
        for part in &self.parts {
            part.source.instance.hash(&mut hasher);
            part.source.key.hash(&mut hasher);
            part.source.signature.hash(&mut hasher);
            part.reference.hash(&mut hasher);
            part.pads.is_some().hash(&mut hasher);
        }
        hasher.finish()
    }
    /// Whether the board has fallen behind the sheet — a component wants a placement it
    /// has not got, or a placement belongs to a component that is gone — and, when it
    /// has, one hash over the two rosters, so a state that has been healed once can be
    /// recognised if it comes back.
    ///
    /// The one question the sweep answers, asked without running it. `None` again the
    /// moment a pass has run, which is what keeps the pass inside [`Editor::commit`]
    /// from running a second sweep inside the first.
    fn stale_mark(&self) -> Option<u64> {
        use std::hash::{Hash, Hasher};
        if self.wiring() {
            return None;
        }
        let mut wanted: Vec<Uuid> = self
            .document
            .components
            .iter()
            .filter(|c| c.pads.is_some() && needs_footprint(c))
            .map(|c| c.id)
            .collect();
        let mut placed: Vec<Uuid> = self
            .document
            .board
            .placements
            .iter()
            .map(|p| p.component)
            .collect();
        wanted.sort();
        placed.sort();
        if wanted == placed {
            return None;
        }
        let mut hasher = std::hash::DefaultHasher::new();
        wanted.hash(&mut hasher);
        placed.hash(&mut hasher);
        Some(hasher.finish())
    }
    pub(crate) fn board_is_stale(&self) -> bool {
        self.stale_mark().is_some()
    }
    /// Put a pass of the sync to the user, in the board view where the board is.
    pub(crate) fn announce(&mut self, sync: &BoardSync, refused: &[String]) {
        let mut message = String::new();
        if sync.added > 0 || sync.removed > 0 {
            message = format!(
                "Board updated from parts: {} added, {} removed.",
                sync.added, sync.removed
            );
            self.board_view.fitted = false;
        }
        // A pass that had to grow the outline says so: the board's size is what a
        // fabricator is quoted, and a fitted board that grew in silence was news only
        // in the Gerbers (the eCAD re-audit's B10).
        // On the sheet too, where a part is usually added and the board's line
        // cannot be seen, as the parts with no pads are named there.
        if let Some((from, to)) = sync.outline_grew {
            let grew = format!(
                "No free room was left on the board, so its outline grew from {} to {} mm.",
                size_words(from),
                size_words(to)
            );
            message = format!("{message} {grew}").trim_start().to_owned();
            self.notice = Some(grew);
            self.board_view.outline = None;
        }
        // Each refusal a sentence of its own: two ran together with no stop
        // read as one garbled line.
        for refusal in refused {
            let stop = if refusal.ends_with(['.', '!', '?']) { "" } else { "." };
            message = format!("{message} {refusal}{stop}").trim_start().to_owned();
        }
        if !message.is_empty() {
            self.board_view.message = message;
        }
    }
    /// The components that want a place on the board and have no pads to take one with,
    /// in natural order — [`BoardSync::without_pads`], computed here rather than taken
    /// from the sweep, because a component with no pads never makes the board STALE and
    /// so never runs one.
    fn without_pads(&self) -> Vec<String> {
        let mut refs: Vec<String> = self
            .document
            .components
            .iter()
            .filter(|c| c.pads.is_none() && needs_footprint(c))
            .map(|c| c.reference.clone())
            .collect();
        refs.sort_by(|a, b| footprint::natural_cmp(a, b));
        refs
    }
    /// Name the components that cannot be placed, ONCE each, wherever the user is
    /// looking: in the board view's own line, and on the sheet, where the automatic
    /// pass usually runs and where the board's line cannot be seen.
    ///
    /// Once, not once per edit: a part with no pads is a standing state and the sheet
    /// is edited all day — the rule `BlockSync::unreported` keeps for an UNLINKED
    /// component. A component that gains pads and loses them again is named again,
    /// which is right: it is news both times. The forced pass restates the lot.
    fn announce_without_pads(&mut self) {
        let standing = self.without_pads();
        if standing == self.reported_without_pads {
            return;
        }
        let fresh: Vec<&String> = standing
            .iter()
            .filter(|r| !self.reported_without_pads.contains(r))
            .collect();
        self.reported_without_pads = standing.clone();
        if fresh.is_empty() {
            return;
        }
        let line = format!(
            "Not on the board, because their parts have no pads: {}.",
            fresh
                .iter()
                .map(|r| r.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
        // Once: the forced pass restates the lot over a message that may already
        // hold this very line (the third eCAD audit, B9: it read twice).
        if !self.board_view.message.contains(&line) {
            self.board_view.message = match self.board_view.message.is_empty() {
                true => line.clone(),
                false => format!("{} {line}", self.board_view.message),
            };
        }
        // A standing state, not a refusal: the sheet's line in the text colour, not
        // the warning red a refused edit gets.
        self.note = Some(line);
    }
    /// True while the autorouter is running; the host should avoid replacing documents.
    pub fn routing(&self) -> bool {
        self.board_view.job.is_some()
    }
    fn set_board_tool(&mut self, tool: BoardTool) {
        if self.board_view.tool != tool {
            self.board_view.draft = None;
            self.board_view.zone_draft.clear();
        }
        self.board_view.tool = tool;
        if tool == BoardTool::Route && self.board_view.draft.is_none() {
            self.route_from_selection();
        }
    }
    /// Arm a track at the selected pad or via, as a Route click on it would: the
    /// selection is where the user has said the copper starts. The selection is
    /// spent doing so — a Route click never selects, so a pad left selected would
    /// arm a second track from the same place the next time Route is chosen.
    fn route_from_selection(&mut self) {
        let start = match self.board_view.selected {
            Some(BoardSelection::Pad(id, index)) => self.pad_anchor(id, index),
            Some(BoardSelection::Via(id)) => self
                .document
                .board
                .vias
                .iter()
                .find(|v| v.id == id)
                .map(|v| (v.at, Anchor::Via)),
            _ => None,
        };
        if let Some((at, anchor)) = start {
            self.board_view.tool = BoardTool::Route;
            self.start_draft(at, Some(anchor));
            self.board_view.selected = None;
        }
    }
    /// Where a track from pad `index` of `component` starts, and what it starts on.
    fn pad_anchor(&self, component: Uuid, index: usize) -> Option<(Point, Anchor)> {
        let board = &self.document.board;
        let placement = board.placement(component)?;
        let pad = placement.footprint.pads.get(index)?;
        Some((
            placement.transform(pad.at),
            Anchor::Pad {
                layers: placement.pad_layers(pad, board.layer_count),
                width: self.pad_width(component, &pad.number),
            },
        ))
    }
    /// The track width a route from a pad starts with: its net's width rule, or the
    /// board's default for a pad no net reaches.
    fn pad_width(&self, component: Uuid, number: &str) -> i32 {
        let board = &self.document.board;
        self.document
            .netlist()
            .nets
            .iter()
            .find(|n| {
                n.pins
                    .iter()
                    .any(|pin| pin.component_id == component && pin.number == number)
            })
            .map_or(board.rules.track_width, |n| board.rules.width_for(&n.name))
    }
    fn choose_layer(&mut self, layer: u8) {
        if layer != self.board_view.active_layer {
            self.switch_layer(layer);
        }
    }
    pub(crate) fn zoom_board(&mut self, factor: f32) {
        let centre = self.board_view.canvas.center();
        self.board_view.zoom_about(factor, centre);
    }
    pub(crate) fn fit_board(&mut self) {
        self.board_view.fit(&self.document.board);
    }
    /// Forget everything in progress on the board, including an autoroute run, for a
    /// document loaded by the host.
    pub(crate) fn reset_board_gestures(&mut self) {
        self.cancel_board();
        self.board_view.job = None;
        self.board_view.selected = None;
        self.board_view.filled_from = None;
        self.board_view.opened_stale = false;
        self.board_view.zone_reports.clear();
    }
    pub(crate) fn cancel_board(&mut self) {
        let view = &mut self.board_view;
        if let Some((before, ..)) = view.drag.take() {
            self.document = before;
        }
        view.placing = None;
        view.draft = None;
        view.zone_draft.clear();
        view.rules = None;
        view.outline = None;
        view.violations = None;
        view.tool = BoardTool::Select;
    }
    /// The reference the schematic gives a placed component, or `?` for a
    /// placement whose component the schematic no longer has.
    fn reference_of(&self, component: Uuid) -> &str {
        self.document
            .components
            .iter()
            .find(|c| c.id == component)
            .map_or("?", |c| c.reference.as_str())
    }
    /// One pad of a placed part, with the part's reference. Every read of a
    /// [`BoardSelection::Pad`] comes through here, so an index left behind by a
    /// footprint the host has since refreshed answers `None` — the view draws
    /// nothing for it and the inspector says the pad is gone, rather than
    /// indexing off the end.
    fn pad_at(&self, component: Uuid, index: usize) -> Option<(&str, &Pad)> {
        let pad = self
            .document
            .board
            .placement(component)?
            .footprint
            .pads
            .get(index)?;
        Some((self.reference_of(component), pad))
    }
    /// One silkscreen line of a placed part, in BOARD coordinates, with the
    /// part's reference: [`Self::pad_at`]'s twin, and for the same reason — a
    /// line index left behind by a refreshed footprint answers `None`.
    fn silk_at(&self, component: Uuid, index: usize) -> Option<(&str, Vec<Point>)> {
        let placement = self.document.board.placement(component)?;
        let line = placement.footprint.silk.get(index)?;
        Some((
            self.reference_of(component),
            line.iter().map(|p| placement.transform(*p)).collect(),
        ))
    }
    /// The net the schematic puts on each of one component's pins, by pin
    /// number — the join eCAD makes everywhere: a pad matches the pin of the
    /// same number. Built once for a list of pads rather than once per pad,
    /// because a netlist is derived from the whole sheet.
    fn nets_of(&self, component: Uuid) -> BTreeMap<String, String> {
        let mut out = BTreeMap::new();
        for net in self.document.netlist().nets {
            for pin in net.pins.iter().filter(|pin| pin.component_id == component) {
                out.insert(pin.number.clone(), net.name.clone());
            }
        }
        out
    }
    /// The net on one pad: `None` for a mechanical pad, and for a numbered pad
    /// that no net on the sheet reaches.
    fn pad_net(&self, component: Uuid, number: &str) -> Option<String> {
        if number.is_empty() {
            return None;
        }
        self.nets_of(component).remove(number)
    }
    /// The selected pad as `(its part, the pad NUMBER)`, or `None` when the
    /// selection is not a pad. The board's twin of
    /// [`crate::FootprintEditor::selected_pad`]: the number is the identity a
    /// pad shares with its pin and with a part's connection point, so it is the
    /// whole answer. A MECHANICAL pad answers with an empty number, which is
    /// what it has.
    pub fn selected_board_pad(&self) -> Option<(Uuid, &str)> {
        match self.board_view.selected {
            Some(BoardSelection::Pad(id, index)) => self
                .pad_at(id, index)
                .map(|(_, pad)| (id, pad.number.as_str())),
            _ => None,
        }
    }
    pub fn selected_board_silk(&self) -> Option<(Uuid, usize)> {
        match self.board_view.selected {
            Some(BoardSelection::Silk(id, index)) => Some((id, index)),
            _ => None,
        }
    }
    /// The selected zone's index on the board, or `None` when the selection is not a zone.
    pub fn selected_board_zone(&self) -> Option<usize> {
        match self.board_view.selected {
            Some(BoardSelection::Zone(id)) => self.document.board.zones.iter().position(|z| z.id == id),
            _ => None,
        }
    }
    /// The spokes each relieved pad got in zone `id`'s last fill this session, in
    /// the fill's order, with the pad's name: `None` until it has been filled here.
    pub fn zone_spokes(&self, id: Uuid) -> Option<Vec<(String, u8)>> {
        let report = self.board_view.zone_reports.get(&id)?;
        Some(report.thermals.iter().map(|t| (self.copper_name(t.owner), t.spokes)).collect())
    }
    /// The board's Inspector message, "" when there is none: what the last board action said.
    pub fn board_message(&self) -> &str { &self.board_view.message }
    /// Select pad `number` of the placed component `component`, and say whether
    /// there was one. The [`crate::FootprintEditor::select_pad`] twin, for a
    /// host lighting one connection point up on every surface it appears on. A
    /// pad outside the canvas is brought into view, because a highlight nobody
    /// can see is not one; a pad already on screen does not move the board.
    /// Selecting is not an edit: [`Editor::take_change`] reports nothing.
    pub fn select_board_pad(&mut self, component: Uuid, number: &str) -> bool {
        // A mechanical pad has no name, so nothing can ask for it by one. It is
        // still selected by a click, and still says what it is.
        if number.is_empty() {
            return false;
        }
        let Some(placement) = self.document.board.placement(component) else {
            return false;
        };
        let Some(index) = placement
            .footprint
            .pads
            .iter()
            .position(|pad| pad.number == number)
        else {
            return false;
        };
        let at = placement.transform(placement.footprint.pads[index].at);
        self.board_view.selected = Some(BoardSelection::Pad(component, index));
        if !self
            .board_view
            .canvas
            .shrink(12.)
            .contains(self.board_view.screen(at))
        {
            self.board_view.centre_on(at);
        }
        true
    }
    fn edit_part(&mut self, id: Uuid, edit: impl FnOnce(&mut Placement)) {
        self.transaction(|d| {
            if let Some(p) = d.board.placements.iter_mut().find(|p| p.component == id) {
                edit(p);
            }
        });
    }
    fn rotate_part(&mut self) {
        if let Some(id) = part_of(self.board_view.selected) {
            self.edit_part(id, |p| p.rotation = (p.rotation + 1) % 4);
        }
    }
    fn flip_part(&mut self) {
        if let Some(id) = part_of(self.board_view.selected) {
            if self.document.board.layer_count < 2 {
                self.board_view.message =
                    "Add a second copper layer to mount parts on the bottom.".into();
                return;
            }
            self.edit_part(id, |p| p.bottom = !p.bottom);
        }
    }
    fn delete_board_selection(&mut self) {
        match self.board_view.selected {
            Some(BoardSelection::Track(id)) => {
                self.transaction(|d| d.board.tracks.retain(|t| t.id != id));
                self.board_view.selected = None;
            }
            Some(BoardSelection::Via(id)) => {
                self.transaction(|d| d.board.vias.retain(|v| v.id != id));
                self.board_view.selected = None;
            }
            Some(BoardSelection::Zone(id)) => {
                self.transaction(|d| {
                    d.board.zones.retain(|z| z.id != id);
                    d.fill_zones();
                });
                self.note_filled();
                self.board_view.selected = None;
            }
            Some(BoardSelection::Part(id)) => {
                self.transaction(|d| d.delete_component(id));
                self.board_view.selected = None;
            }
            // A pad is a copy of its part's footprint, refreshed from the part:
            // deleting it here would come back at the next refresh, and deleting
            // the whole part is not what a click on one pad asked for.
            Some(BoardSelection::Pad(id, index)) => {
                let what = self
                    .pad_at(id, index)
                    .map_or_else(|| "That pad".to_owned(), |(r, pad)| pad_title(r, pad));
                self.board_view.message = format!(
                    "{what} belongs to its part's footprint. Edit the pads on the part, in the Pads workbench; Delete here removes a track, a via or a whole part."
                );
            }
            // The same for a silkscreen line, for the same reason.
            Some(BoardSelection::Silk(id, _)) => {
                let reference = self.reference_of(id).to_owned();
                self.board_view.message = format!(
                    "That silkscreen line belongs to {reference}'s footprint. Edit it on the part, in the Pads workbench; Delete here removes a track, a via or a whole part."
                );
            }
            None => {}
        }
    }
    /// The pad a thermal relief is on, as the Inspector names it: "J1.1".
    fn copper_name(&self, owner: CopperRef) -> String {
        match owner {
            CopperRef::Pad { placement, pad } => {
                let Some(placement) = self.document.board.placements.get(placement) else {
                    return "?".into();
                };
                let number = placement.footprint.pads.get(pad).map_or("?", |p| p.number.as_str());
                format!("{}.{number}", self.reference_of(placement.component))
            }
            CopperRef::Via(_) => "via".into(),
            CopperRef::Track { .. } => "track".into(),
        }
    }
    /// Remember the board the zones were just filled from.
    fn note_filled(&mut self) {
        self.board_view.filled_from = Some(self.document.board.clone());
        self.board_view.opened_stale = false;
    }
    /// The first sight of a board with zones since it was loaded: take it as the
    /// baseline later edits are compared with, and ask ONCE whether the fill it
    /// came with is what its copper asks for ([`Board::zones_stale`], a refill of
    /// a copy: about a tenth of a second on a 40 × 30 mm board). A fill saved
    /// stale is marked stale on open, with no edit made and nothing written.
    pub(crate) fn seed_fill_baseline(&mut self) {
        if self.board_view.filled_from.is_some() || self.document.board.zones.is_empty() {
            return;
        }
        self.note_filled();
        self.board_view.opened_stale = self.document.board.zones_stale(&self.document.netlist());
        if self.board_view.opened_stale {
            // Said where the board's line is read, not only drawn on the canvas.
            let line = if self.document.board.zones.iter().all(|z| z.fill.is_empty()) {
                "The zones are not filled yet. Fill zones (B) pours them."
            } else {
                "The zones' fill was saved out of date: copper changed after it was filled, so it may cross a track \
                 or leave a pad cut off. Fill zones (B) refills it; the fabrication and STEP exports refill a copy."
            };
            self.board_view.message = match self.board_view.message.is_empty() {
                true => line.to_owned(),
                false => format!("{} {line}", self.board_view.message),
            };
        }
    }
    /// Whether the zones' fill on show is not the one the copper asks for: it was
    /// opened so, or copper has moved since it was made, and the fill may cross it
    /// or leave a pad cut off.
    /// The design rule check's findings as the Inspector lists them, each by the
    /// word it is listed under and its message; `None` until the check has run
    /// (and again once an edit has hidden them).
    pub fn board_findings(&self) -> Option<Vec<(&'static str, String)>> {
        self.board_view
            .violations
            .as_ref()
            .map(|v| v.iter().map(|v| (violation_kind(v.kind), v.message.clone())).collect())
    }
    pub fn zones_stale(&self) -> bool {
        !self.document.board.zones.is_empty()
            && (self.board_view.opened_stale
                || self
                    .board_view
                    .filled_from
                    .as_ref()
                    .is_some_and(|b| !fill_inputs_match(b, &self.document.board)))
    }
    /// Refill every zone, as one undo step, and say what each fill did.
    pub(crate) fn fill_zones(&mut self) {
        let mut reports = vec![];
        self.transaction(|d| reports = d.fill_zones());
        self.note_filled();
        let words = self.zone_reports_words(&reports);
        self.board_view.message = match words.len() {
            0 => "There are no zones to fill.".into(),
            _ => format!("Zones filled. {}", words.join(" ")),
        };
    }
    /// Each report in a line, remembered for the Inspector.
    fn zone_reports_words(&mut self, reports: &[ZoneReport]) -> Vec<String> {
        let count = self.document.board.layer_count;
        let mut words = vec![];
        for report in reports {
            if let Some(zone) = self.document.board.zones.iter().find(|z| z.id == report.zone) {
                words.push(zone_words(zone, report, count, &|owner| self.copper_name(owner)));
            }
        }
        for report in reports {
            self.board_view.zone_reports.insert(report.zone, report.clone());
        }
        words
    }
    /// The nets a zone can pour, in the netlist's order.
    fn net_names(&self) -> Vec<String> {
        self.document.netlist().nets.into_iter().map(|n| n.name).collect()
    }
    /// The net the next zone pours: the one chosen, else GND, else the first net.
    fn zone_net(&self) -> Option<String> {
        let nets = self.net_names();
        if nets.contains(&self.board_view.zone_net) {
            return Some(self.board_view.zone_net.clone());
        }
        nets.iter()
            .find(|n| n.eq_ignore_ascii_case("GND"))
            .or(nets.first())
            .cloned()
    }
    /// Close the zone being drawn, on the active layer, pouring the chosen net;
    /// fill it; and select it, so its corners can be dragged straight away.
    fn finish_zone(&mut self) {
        let corners = std::mem::take(&mut self.board_view.zone_draft);
        if corners.len() < 3 {
            return;
        }
        let Some(net) = self.zone_net() else {
            self.board_view.message =
                "A zone pours a net, and the schematic has none yet: connect some pins first.".into();
            return;
        };
        let layer = self.board_view.active_layer.min(self.document.board.layer_count - 1);
        let zone = Zone::new(&net, layer, corners);
        if zones::ring_area(&zone.outline) == 0. {
            self.board_view.message = "A zone needs corners that enclose an area.".into();
            return;
        }
        let id = zone.id;
        let mut reports = vec![];
        self.transaction(|d| {
            d.board.zones.push(zone);
            reports = d.fill_zones();
        });
        self.note_filled();
        let words = self.zone_reports_words(&reports);
        self.board_view.tool = BoardTool::Select;
        self.board_view.selected = Some(BoardSelection::Zone(id));
        let index = self.document.board.zones.iter().position(|z| z.id == id);
        self.board_view.message = index
            .and_then(|k| words.get(k).cloned())
            .unwrap_or_default();
    }
    /// A Zone-tool click: a corner at the grid point under the pointer, or, on the
    /// first corner with three already down, the zone closed.
    fn zone_click(&mut self, p: Pos2) {
        let q = snap(self.board_view.world(p), PLACE_GRID);
        let draft = &self.board_view.zone_draft;
        if draft.len() >= 3 && self.board_view.screen(draft[0]).distance(p) <= 8. {
            self.finish_zone();
            return;
        }
        if draft.last() != Some(&q) {
            self.board_view.zone_draft.push(q);
        }
    }
    /// What a press at `q` takes of the SELECTED zone: a corner, an edge's middle,
    /// or the zone.
    fn zone_grip(&self, id: Uuid, p: Pos2) -> Option<ZoneGrip> {
        let zone = self.document.board.zones.iter().find(|z| z.id == id)?;
        let view = &self.board_view;
        let near = |q: Point| view.screen(q).distance(p) <= 7.;
        if let Some(i) = zone.outline.iter().position(|&q| near(q)) {
            return Some(ZoneGrip::Corner(i));
        }
        if view.selected == Some(BoardSelection::Zone(id))
            && let Some(i) = edge_middles(&zone.outline).into_iter().position(near)
        {
            return Some(ZoneGrip::Edge(i));
        }
        Some(ZoneGrip::Body)
    }
    fn start_autoroute(&mut self) {
        let netlist = self.document.netlist();
        self.cancel_board();
        self.board_view.job = Some(RouteJob::new(&self.document.board, &netlist));
    }
    fn poll_autoroute(&mut self, ctx: &egui::Context) {
        let Some(mut job) = self.board_view.job.take() else {
            return;
        };
        let progress = job.step(ROUTE_BUDGET);
        ctx.request_repaint();
        let mut cancel = false;
        egui::Modal::new(egui::Id::new("autoroute-progress")).show(ctx, |ui| {
            ui.set_min_width(320.);
            ui.heading("Autorouting");
            let settled = progress.completed_nets + progress.failed_nets;
            let fraction = if progress.total_nets == 0 {
                1.
            } else {
                settled as f32 / progress.total_nets as f32
            };
            ui.add(egui::ProgressBar::new(fraction).show_percentage());
            ui.label(format!(
                "{settled} of {} nets · {} rip-ups · {:.2} mm grid",
                progress.total_nets,
                progress.rip_ups,
                mm(job.pitch())
            ));
            if progress.removing_vias {
                ui.label("Rerouting nets to remove vias they no longer need…");
            }
            if ui.button("Cancel autoroute").clicked() {
                cancel = true;
            }
        });
        if cancel {
            self.board_view.message = "Autoroute cancelled; no copper was added.".into();
        } else if progress.done {
            let outcome = job.outcome();
            let (tracks, vias) = (outcome.tracks.len(), outcome.vias.len());
            self.board_view.message = if outcome.unrouted.is_empty() {
                format!("Autoroute complete: {tracks} tracks and {vias} vias added.")
            } else {
                format!(
                    "Autoroute added {tracks} tracks and {vias} vias. Unrouted nets: {}. Try moving parts apart, a finer grid, or narrower tracks.",
                    outcome.unrouted.join(", ")
                )
            };
            self.transaction(|d| outcome.apply(&mut d.board));
        } else {
            self.board_view.job = Some(job);
        }
    }

    pub(crate) fn board_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            self.view_switch(ui);
            self.action_buttons(ui, ActionGroup::Tool);
            ui.separator();
            self.action_buttons(ui, ActionGroup::History);
            ui.separator();
            self.action_buttons(ui, ActionGroup::Zoom);
            ui.separator();
            let count = self.document.board.layer_count;
            let mut layer = self.board_view.active_layer.min(count - 1);
            let layers: Vec<&'static Action<Editor>> = crate::actions::actions()
                .filter(|a| a.group == ActionGroup::Layer && a.offered(self))
                .collect();
            egui::ComboBox::from_id_salt("active-layer")
                .selected_text(layer_name(layer, count))
                .show_ui(ui, |ui| {
                    for (l, action) in (0..).zip(&layers) {
                        ui.selectable_value(&mut layer, l, action.caption(self));
                    }
                });
            if layer != self.board_view.active_layer {
                self.run_action(layers[usize::from(layer)].id);
            }
        });
        ui.horizontal_wrapped(|ui| {
            match self.board_view.tool {
                BoardTool::Select => {
                    ui.label("Drag parts and vias to place them, and a track by a corner or a segment to reshape it — its ends stay on the pads and vias they meet, and a free end dropped on a pad of its net snaps to the pad's centre. Select a pad and press X to route from it. R rotates, right-drag pans.");
                }
                BoardTool::Route => {
                    ui.label(if self.board_view.draft.is_some() {
                        "Click to add corners; click a pad or copper to finish."
                    } else {
                        "Click a pad to start a track."
                    });
                }
                BoardTool::Zone => {
                    ui.label("Click the zone's corners; click the first again, double-click or press Enter to close it. It pours the net chosen in the Inspector.");
                }
            }
            self.action_buttons(ui, ActionGroup::Context);
        });
    }

    pub(crate) fn board_panel(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.);
        ui.heading("Assembly parts");
        if ui.button("Update from parts").clicked() {
            self.sync_board();
        }
        let references: BTreeMap<Uuid, (String, String)> = self
            .document
            .components
            .iter()
            .map(|c| (c.id, (c.reference.clone(), c.value.clone())))
            .collect();
        let mut parts: Vec<(String, Uuid, String)> = self
            .document
            .board
            .placements
            .iter()
            .map(|p| {
                let reference = references.get(&p.component).map_or("?", |r| r.0.as_str());
                let package = p.footprint.name.rsplit(':').next().unwrap_or("");
                (reference.to_owned(), p.component, package.to_owned())
            })
            .collect();
        parts.sort_by(|a, b| footprint::natural_cmp(&a.0, &b.0));
        let mut chosen = None;
        egui::ScrollArea::vertical()
            .id_salt("board-parts")
            .max_height((ui.available_height() * 0.55).max(80.))
            .show(ui, |ui| {
                if parts.is_empty() {
                    ui.small("Add native parts with pads to the assembly, then update the board.");
                }
                for (reference, id, package) in &parts {
                    // A pad's part is lit here too: the selection is one thing seen
                    // from two distances, and a list that went blank when a pad was
                    // picked would read as having lost it.
                    let selected = part_of(self.board_view.selected) == Some(*id);
                    let response =
                        ui.selectable_label(selected, format!("{reference}  ·  {package}"));
                    if response.clicked() {
                        chosen = Some(*id);
                    }
                }
            });
        if let Some(id) = chosen {
            self.board_view.selected = Some(BoardSelection::Part(id));
            if let Some(p) = self.document.board.placement(id) {
                let at = p.at;
                self.board_view.centre_on(at);
            }
        }
        ui.separator();
        ui.heading("Layers");
        let count = self.document.board.layer_count;
        for layer in 0..count {
            ui.horizontal(|ui| {
                let (swatch, _) = ui.allocate_exact_size(Vec2::splat(12.), Sense::hover());
                ui.painter()
                    .rect_filled(swatch, 2., layer_color(layer, count));
                let mut visible = self.board_view.visible(layer);
                if ui
                    .checkbox(&mut visible, layer_name(layer, count))
                    .changed()
                {
                    if visible {
                        self.board_view.hidden.remove(&layer);
                    } else {
                        self.board_view.hidden.insert(layer);
                    }
                }
                if self.board_view.active_layer == layer {
                    ui.small(egui::RichText::new("active").color(accent(ui)));
                }
            });
        }
        ui.checkbox(&mut self.board_view.show_ratsnest, "Ratsnest");
    }

    pub(crate) fn board_inspector(&mut self, ui: &mut egui::Ui) {
        self.board_view.inspector.begin(ui.ctx());
        ui.add_space(12.);
        ui.heading("Properties");
        ui.add_space(8.);
        match self.board_view.selected {
            Some(BoardSelection::Part(id)) => self.part_inspector(ui, id),
            Some(BoardSelection::Pad(id, index)) => self.pad_inspector(ui, id, index),
            Some(BoardSelection::Silk(id, index)) => self.silk_inspector(ui, id, index),
            Some(BoardSelection::Track(id)) => {
                if let Some(t) = self
                    .document
                    .board
                    .tracks
                    .iter()
                    .find(|t| t.id == id)
                    .cloned()
                {
                    let length: f64 = t
                        .points
                        .windows(2)
                        .map(|s| f64::from(s[1].x - s[0].x).hypot(f64::from(s[1].y - s[0].y)))
                        .sum();
                    let net = self.document.track_nets().get(&id).cloned();
                    ui.label(egui::RichText::new("Track").color(accent(ui)));
                    ui.small(format!(
                        "{} · {:.2} mm long · {}",
                        layer_name(t.layer, self.document.board.layer_count),
                        length / 1000.,
                        net.as_deref().map_or_else(
                            || "no net".to_owned(),
                            |n| format!("{n} ({})", self.document.board.rules.class_of(Some(n)).name)
                        )
                    ));
                    ui.horizontal(|ui| {
                        ui.label("Width");
                        let mut width = mm(t.width);
                        let field = ui
                            .add(
                                egui::DragValue::new(&mut width)
                                    .speed(0.01)
                                    .range(0.05..=5.0)
                                    .fixed_decimals(3)
                                    .suffix(" mm"),
                            )
                            .on_hover_text("Run the design rule check after widening copper.");
                        self.board_view.inspector.mark("track_width", &field);
                        if field.changed() {
                            self.transaction_as(Some(format!("track:{id}:width")), |d| {
                                let _ = d.set_track_width(id, um(width));
                            });
                        }
                        if t.fixed_width {
                            ui.small(egui::RichText::new("set here").color(accent(ui)));
                        }
                    });
                    if t.fixed_width {
                        let follow = ui
                            .button(match &net {
                                Some(net) => format!("Follow {net}'s width"),
                                None => "Follow the default width".into(),
                            })
                            .on_hover_text("Hand this track back to its net's width rule.");
                        self.board_view.inspector.mark("follow_net_width", &follow);
                        if follow.clicked() {
                            self.transaction(|d| d.follow_net_width(id));
                        }
                    }
                    let for_net = net.as_ref().map(|net| {
                        ui.button(format!("Use this width for {net}")).on_hover_text(
                            "Set the net's width rule and give every track on it this width.",
                        )
                    });
                    if let Some(button) = &for_net {
                        self.board_view.inspector.mark("width_for_net", button);
                    }
                    if let Some(net) = net
                        && for_net.is_some_and(|b| b.clicked())
                    {
                        let width = t.width;
                        let mut changed = 0;
                        self.transaction(|d| {
                            d.board.rules.net_widths.insert(net.clone(), width);
                            changed = d.apply_net_widths();
                        });
                        self.board_view.message = format!(
                            "{net} is {:.3} mm wide: {changed} tracks changed.",
                            mm(width)
                        );
                    }
                    let delete = ui.button("Delete track");
                    self.board_view.inspector.mark("delete", &delete);
                    if delete.clicked() {
                        self.delete_board_selection();
                    }
                }
            }
            Some(BoardSelection::Via(id)) => {
                if let Some(v) = self.document.board.vias.iter().find(|v| v.id == id) {
                    ui.label(egui::RichText::new("Via").color(accent(ui)));
                    ui.small(format!(
                        "{:.2} mm pad · {:.2} mm drill at {:.2}, {:.2} mm",
                        mm(v.diameter),
                        mm(v.drill),
                        mm(v.at.x),
                        mm(v.at.y)
                    ));
                    let delete = ui.button("Delete via");
                    self.board_view.inspector.mark("delete", &delete);
                    if delete.clicked() {
                        self.delete_board_selection();
                    }
                }
            }
            Some(BoardSelection::Zone(id)) => self.zone_inspector(ui, id),
            None if self.board_view.tool == BoardTool::Zone => self.new_zone_inspector(ui),
            None => {
                ui.label(
                    "Select a part, a pad, a silkscreen line, a track, a via or a zone to see it here.",
                );
            }
        }
        if !self.board_view.message.is_empty() {
            ui.add_space(6.);
            ui.label(
                egui::RichText::new(&self.board_view.message)
                    .small()
                    .color(accent(ui)),
            );
        }
        ui.add_space(10.);
        ui.separator();
        egui::ScrollArea::vertical()
            .id_salt("board-setup")
            .show(ui, |ui| {
                self.routing_section(ui);
                ui.separator();
                self.drc_section(ui);
                ui.separator();
                let outline = egui::CollapsingHeader::new("Board outline and layers")
                    .default_open(true)
                    .show(ui, |ui| self.outline_section(ui));
                self.board_view.inspector.mark("outline_section", &outline.header_response);
                let rules = egui::CollapsingHeader::new("Design rules")
                    .default_open(false)
                    .show(ui, |ui| self.rules_section(ui));
                self.board_view.inspector.mark("rules_section", &rules.header_response);
                let classes = egui::CollapsingHeader::new("Net classes")
                    .default_open(false)
                    .show(ui, |ui| self.net_classes_section(ui));
                self.board_view.inspector.mark("net_classes_section", &classes.header_response);
            });
    }
    /// The net picker a zone is given its net with: every net on the sheet.
    fn net_combo(&mut self, ui: &mut egui::Ui, salt: &str, current: &str) -> Option<String> {
        let mut chosen = None;
        let combo = egui::ComboBox::from_id_salt(salt)
            .selected_text(if current.is_empty() { "no net" } else { current })
            .show_ui(ui, |ui| {
                for net in self.net_names() {
                    if ui.selectable_label(net == current, &net).clicked() {
                        chosen = Some(net);
                    }
                }
            });
        self.board_view.inspector.mark(salt, &combo.response);
        chosen.filter(|n| n != current)
    }
    /// The Inspector while the Zone tool is in hand and nothing is selected: the
    /// net the zone will pour and the layer it goes on, before its first corner.
    fn new_zone_inspector(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("New zone").color(accent(ui)));
        let count = self.document.board.layer_count;
        let layer = layer_name(self.board_view.active_layer.min(count - 1), count);
        let Some(net) = self.zone_net() else {
            ui.small("The schematic has no nets yet: connect some pins, then draw the zone.");
            return;
        };
        ui.horizontal(|ui| {
            ui.label("Net");
            if let Some(net) = self.net_combo(ui, "zone_net", &net) {
                self.board_view.zone_net = net;
            }
        });
        ui.small(format!(
            "On {layer}, the active copper layer. {} corner(s) placed; the zone is filled as soon as it is closed.",
            self.board_view.zone_draft.len()
        ));
        if self.board_view.zone_draft.len() >= 3 {
            let close = ui.button("Close zone");
            self.board_view.inspector.mark("close_zone", &close);
            if close.clicked() {
                self.finish_zone();
            }
        }
    }
    /// A selected zone: what its fill did, its net, its layer and its fill rules,
    /// each change refilled at once.
    fn zone_inspector(&mut self, ui: &mut egui::Ui, id: Uuid) {
        let Some(zone) = self.document.board.zones.iter().find(|z| z.id == id).cloned() else {
            return;
        };
        let count = self.document.board.layer_count;
        ui.label(egui::RichText::new("Zone").color(accent(ui)));
        let holes: usize = zone.fill.iter().map(|p| p.holes.len()).sum();
        // An empty fill is "no fill", not "-0.0 mm² filled in 0 piece(s)".
        let fill = if zone.fill.is_empty() {
            "no fill".to_owned()
        } else {
            format!("{:.1} mm² filled in {} piece(s), {holes} hole(s)", zone.fill_area() / 1e6, zone.fill.len())
        };
        ui.small(format!("{} · {} corners · {fill}", zones::describe(&zone, count), zone.outline.len()));
        // The last fill's thermal reliefs, unless the line under the Inspector
        // is saying exactly that already.
        if let Some(report) = self.board_view.zone_reports.get(&id).cloned() {
            let words = zone_words(&zone, &report, count, &|owner| self.copper_name(owner));
            if !self.board_view.message.contains(&words) {
                ui.small(words);
            }
        }
        if self.zones_stale() {
            ui.label(
                egui::RichText::new(
                    "The fill is out of date: copper has changed since it was filled. Fill zones (B) refills it; so does the design rule check.",
                )
                .small()
                .color(WARNING),
            );
        }
        let mut edit: Option<Box<dyn FnOnce(&mut Zone)>> = None;
        let mut settle = false;
        ui.horizontal(|ui| {
            ui.label("Net");
            if let Some(net) = self.net_combo(ui, "zone_net", &zone.net) {
                edit = Some(Box::new(move |z| z.net = net));
                settle = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Layer");
            let mut layer = zone.layer;
            let combo = egui::ComboBox::from_id_salt("zone_layer")
                .selected_text(layer_name(layer, count))
                .show_ui(ui, |ui| {
                    for l in 0..count {
                        ui.selectable_value(&mut layer, l, layer_name(l, count));
                    }
                });
            self.board_view.inspector.mark("zone_layer", &combo.response);
            if layer != zone.layer {
                edit = Some(Box::new(move |z| z.layer = layer));
                settle = true;
            }
        });
        for (key, label, value, hint) in [
            ("thermal_gap", "Thermal gap", zone.thermal_gap, "The gap round a through-hole pad of the zone's net, bridged by spokes. At least the clearance of the net's class."),
            ("spoke_width", "Spoke width", zone.spoke_width, "The width of each thermal spoke."),
            ("min_width", "Minimum width", zone.min_width, "Fill narrower than this is left out."),
        ] {
            ui.horizontal(|ui| {
                ui.label(label);
                let mut v = mm(value);
                let field = ui
                    .add(
                        egui::DragValue::new(&mut v)
                            .speed(0.01)
                            .range(0.05..=10.0)
                            .fixed_decimals(3)
                            .suffix(" mm"),
                    )
                    .on_hover_text(hint);
                self.board_view.inspector.mark(key, &field);
                if field.changed() && um(v) != value {
                    let v = um(v);
                    edit = Some(Box::new(move |z| match key {
                        "thermal_gap" => z.thermal_gap = v,
                        "spoke_width" => z.spoke_width = v,
                        _ => z.min_width = v,
                    }));
                    // A drag refills once, when it is let go; a typed value or an
                    // arrow key at once.
                    settle = !field.dragged();
                }
                if field.drag_stopped() {
                    settle = true;
                }
            });
        }
        if let Some(edit) = edit {
            self.transaction_as(Some(format!("zone:{id}:settings")), |d| {
                if let Some(z) = d.board.zones.iter_mut().find(|z| z.id == id) {
                    edit(z);
                }
            });
        }
        if settle {
            self.fill_zones();
        }
        ui.horizontal(|ui| {
            let fill = ui
                .button("Fill zones")
                .on_hover_text("Refill every zone against the copper as it is now (B).");
            self.board_view.inspector.mark("fill_zones", &fill);
            if fill.clicked() {
                self.fill_zones();
            }
            let delete = ui.button("Delete zone");
            self.board_view.inspector.mark("delete", &delete);
            if delete.clicked() {
                self.delete_board_selection();
            }
        });
    }
    fn part_inspector(&mut self, ui: &mut egui::Ui, id: Uuid) {
        let Some(placement) = self.document.board.placement(id).cloned() else {
            return;
        };
        let (reference, value) = self
            .document
            .components
            .iter()
            .find(|c| c.id == id)
            .map_or_else(Default::default, |c| (c.reference.clone(), c.value.clone()));
        ui.label(egui::RichText::new(format!("{reference} · {value}")).color(accent(ui)));
        ui.small(format!(
            "{} pads · {:.2}, {:.2} mm · {}° · {}",
            placement.footprint.pads.len(),
            mm(placement.at.x),
            mm(placement.at.y),
            u16::from(placement.rotation) * 90,
            if placement.bottom { "bottom" } else { "top" }
        ));
        ui.horizontal(|ui| {
            let rotate = ui.button("Rotate 90°");
            self.board_view.inspector.mark("rotate", &rotate);
            if rotate.clicked() {
                self.rotate_part();
            }
            let flip = ui.button(if placement.bottom {
                "Move to top"
            } else {
                "Move to bottom"
            });
            self.board_view.inspector.mark("flip", &flip);
            if flip.clicked() {
                self.flip_part();
            }
        });
        // A placement's pads are a copy of its part's, edited on the part.
        ui.label("Footprint");
        ui.small(if placement.footprint.name.is_empty() {
            "(unnamed)"
        } else {
            placement.footprint.name.as_str()
        });
        // Its pads, each with the net that reaches it. A pad is selectable here
        // as well as on the canvas, because a pad under a track, or smaller than
        // a pointer, is a poor target and is exactly the one a user wants when
        // they are chasing a connection.
        ui.add_space(6.);
        ui.label("Pads");
        let nets = self.nets_of(id);
        let selected = self.board_view.selected;
        let mut pick = None;
        egui::ScrollArea::vertical()
            .id_salt("board-part-pads")
            .max_height(180.)
            .show(ui, |ui| {
                for (index, pad) in placement.footprint.pads.iter().enumerate() {
                    let on = selected == Some(BoardSelection::Pad(id, index));
                    let text = match (pad.number.as_str(), nets.get(&pad.number)) {
                        ("", _) => "mechanical".to_owned(),
                        (number, Some(net)) => format!("{number}  ·  {net}"),
                        (number, None) => format!("{number}  ·  no net"),
                    };
                    let row = ui.selectable_label(on, text);
                    // By the key the pad has on the canvas; a mechanical pad, which
                    // has none there, by its index in the footprint.
                    match pad.number.as_str() {
                        "" => self.board_view.inspector.mark(format!("row:unnumbered-pad:{reference}.{index}"), &row),
                        number => self.board_view.inspector.mark(format!("row:pad:{reference}.{number}"), &row),
                    }
                    if row.clicked() {
                        pick = Some(index);
                    }
                }
            });
        if placement.footprint.pads.is_empty() {
            ui.small("This footprint has no pads.");
        }
        if let Some(index) = pick {
            self.board_view.selected = Some(BoardSelection::Pad(id, index));
        }
    }
    /// One PAD of a placed part: what it is, which net reaches it, where it sits,
    /// and where it is edited — which is the part it came from, never here.
    fn pad_inspector(&mut self, ui: &mut egui::Ui, id: Uuid, index: usize) {
        let Some(placement) = self.document.board.placement(id).cloned() else {
            return;
        };
        let reference = self.reference_of(id).to_owned();
        let Some(pad) = placement.footprint.pads.get(index).cloned() else {
            // The footprint was refreshed from the part under the selection.
            ui.small(format!("That pad is no longer in {reference}'s footprint."));
            let select = ui.button(format!("Select {reference}"));
            self.board_view.inspector.mark("select_part", &select);
            if select.clicked() {
                self.board_view.selected = Some(BoardSelection::Part(id));
            }
            return;
        };
        let count = self.document.board.layer_count;
        ui.label(egui::RichText::new(pad_title(&reference, &pad)).color(accent(ui)));
        match (pad.number.is_empty(), self.pad_net(id, &pad.number)) {
            (true, _) => {
                ui.small(
                    "No pin and no net: a mechanical pad reaches the copper and drill files and never the netlist.",
                );
            }
            (false, Some(net)) => {
                ui.small(format!("Net {net}, from pin {} on the sheet.", pad.number));
            }
            (false, None) => {
                ui.label(
                    egui::RichText::new(format!(
                        "No net reaches pin {}. Wire it on the sheet, or make the pad mechanical on the part.",
                        pad.number
                    ))
                    .small()
                    .color(ui.visuals().warn_fg_color),
                );
            }
        }
        let (first, last) = placement.pad_layers(&pad, count);
        let layers = if first == last {
            layer_name(first, count).to_owned()
        } else {
            format!(
                "{} to {}",
                layer_name(first, count),
                layer_name(last, count)
            )
        };
        let at = placement.transform(pad.at);
        ui.small(format!(
            "{} · {} · {:.2} × {:.2} mm · at {:.2}, {:.2} mm",
            match pad.drill {
                None => "Surface mount".to_owned(),
                Some(drill) if pad.plated =>
                    format!("Plated through hole, {:.2} mm drill", mm(drill)),
                Some(drill) => format!("Unplated hole, {:.2} mm drill", mm(drill)),
            },
            layers,
            mm(pad.size.x),
            mm(pad.size.y),
            mm(at.x),
            mm(at.y),
        ));
        ui.add_space(6.);
        ui.horizontal_wrapped(|ui| {
            let select = ui.button(format!("Select {reference}"));
            self.board_view.inspector.mark("select_part", &select);
            if select.clicked() {
                self.board_view.selected = Some(BoardSelection::Part(id));
            }
            let rotate = ui.button("Rotate 90°").on_hover_text("Turns the whole part, which is what a pad turns with.");
            self.board_view.inspector.mark("rotate", &rotate);
            if rotate.clicked() {
                self.rotate_part();
            }
        });
        ui.small("A pad's size, number and drill belong to the part's footprint: edit them in the Pads workbench.");
    }
    /// One SILKSCREEN line of a placed part: which part's, how long, and where
    /// it is edited — the part's footprint, as for a pad.
    fn silk_inspector(&mut self, ui: &mut egui::Ui, id: Uuid, index: usize) {
        let reference = self.reference_of(id).to_owned();
        let Some((_, line)) = self.silk_at(id, index) else {
            ui.small(format!(
                "That silkscreen line is no longer in {reference}'s footprint."
            ));
            let select = ui.button(format!("Select {reference}"));
            self.board_view.inspector.mark("select_part", &select);
            if select.clicked() {
                self.board_view.selected = Some(BoardSelection::Part(id));
            }
            return;
        };
        let side = match self.document.board.placement(id) {
            Some(p) if p.bottom => "bottom",
            _ => "top",
        };
        let length: f64 = line
            .windows(2)
            .map(|s| f64::from(s[1].x - s[0].x).hypot(f64::from(s[1].y - s[0].y)))
            .sum();
        ui.label(
            egui::RichText::new(format!("{reference} · silkscreen line {index}")).color(accent(ui)),
        );
        ui.small(format!(
            "{} points · {:.2} mm long · {side} silkscreen",
            line.len(),
            length / 1000.
        ));
        ui.small("Decoration: it prints on the silkscreen film. It carries no pin and no net.");
        ui.add_space(6.);
        ui.horizontal_wrapped(|ui| {
            let select = ui.button(format!("Select {reference}"));
            self.board_view.inspector.mark("select_part", &select);
            if select.clicked() {
                self.board_view.selected = Some(BoardSelection::Part(id));
            }
            let rotate = ui.button("Rotate 90°").on_hover_text("Turns the whole part, which is what its silkscreen turns with.");
            self.board_view.inspector.mark("rotate", &rotate);
            if rotate.clicked() {
                self.rotate_part();
            }
        });
        ui.small("A footprint's silkscreen is edited on the part, in the Pads workbench. Dragging the line here moves the part.");
    }
    fn routing_section(&mut self, ui: &mut egui::Ui) {
        ui.heading("Routing");
        let board = &self.document.board;
        ui.small(format!(
            "{} parts · {} tracks · {} vias · {} unrouted connections",
            board.placements.len(),
            board.tracks.len(),
            board.vias.len(),
            self.board_view.unrouted
        ));
        let has_copper = !board.tracks.is_empty() || !board.vias.is_empty();
        let current = board.rules.autoroute_vias;
        let mut policy = current;
        ui.add_enabled_ui(!self.routing(), |ui| {
            ui.horizontal(|ui| {
                ui.label("Vias");
                let combo = egui::ComboBox::from_id_salt("autoroute-vias")
                    .selected_text(via_policy_name(policy))
                    .show_ui(ui, |ui| {
                        for (option, hint) in [
                            (
                                ViaPolicy::Avoid,
                                "Route as Allow does, then reroute each net that has vias, keeping a route without them wherever one exists. Connects the same nets as Allow; takes longer.",
                            ),
                            (
                                ViaPolicy::Allow,
                                "Use a via whenever it makes the route noticeably shorter.",
                            ),
                            (
                                ViaPolicy::Never,
                                "Keep every connection on one layer. Connections that need a via stay unrouted.",
                            ),
                        ] {
                            let choice = ui
                                .selectable_value(&mut policy, option, via_policy_name(option))
                                .on_hover_text(hint);
                            self.board_view.inspector.mark(
                                format!("vias:{}", key_word(via_policy_name(option).split(' ').next().unwrap_or(""))),
                                &choice,
                            );
                        }
                    });
                let combo = combo
                    .response
                    .on_hover_text("How the autorouter uses vias. Saved with the board.");
                self.board_view.inspector.mark("vias", &combo);
            });
        });
        if policy != current {
            self.transaction(|d| d.board.rules.autoroute_vias = policy);
            // An unapplied edit under Design rules would otherwise put the old choice back.
            if let Some(pending) = &mut self.board_view.rules {
                pending.autoroute_vias = policy;
            }
        }
        ui.horizontal_wrapped(|ui| {
            let autoroute = ui
                .add_enabled(
                    self.board_view.unrouted > 0 && !self.routing(),
                    egui::Button::new("Autoroute"),
                )
                .on_hover_text("Connect every unrouted connection. Existing copper is kept.");
            self.board_view.inspector.mark("autoroute", &autoroute);
            if autoroute.clicked() {
                self.start_autoroute();
            }
            let widths = ui
                .add_enabled(
                    has_copper
                        && (!self.document.board.rules.net_widths.is_empty()
                            || !self.document.board.rules.net_classes.is_empty()),
                    egui::Button::new("Apply net widths"),
                )
                .on_hover_text(
                    "Give every drawn track the width its net asks for: its own width rule, else its net class's. Run the design rule check afterwards.",
                );
            self.board_view.inspector.mark("apply_net_widths", &widths);
            if widths.clicked() {
                let mut changed = 0;
                self.transaction(|d| changed = d.apply_net_widths());
                self.board_view.message = format!("{changed} tracks took their net's width.");
            }
            let rip_up = ui.add_enabled(has_copper, egui::Button::new("Rip up all copper"));
            self.board_view.inspector.mark("rip_up", &rip_up);
            if rip_up.clicked() {
                self.transaction(|d| {
                    d.board.tracks.clear();
                    d.board.vias.clear();
                });
                self.board_view.selected = None;
            }
        });
    }
    fn drc_section(&mut self, ui: &mut egui::Ui) {
        ui.heading("Design rule check");
        let run = ui.button("Run design rule check");
        self.board_view.inspector.mark("run_drc", &run);
        if run.clicked() {
            // The zones are refilled first: the check is of the copper the board
            // would be made with, and that is a fresh fill.
            if !self.document.board.zones.is_empty() {
                let mut reports = vec![];
                self.transaction(|d| reports = d.fill_zones());
                self.note_filled();
                self.zone_reports_words(&reports);
            }
            self.board_view.violations = Some(self.document.board_drc());
            self.board_view.focused_violation = None;
        }
        let Some(violations) = &self.board_view.violations else {
            return;
        };
        if violations.is_empty() {
            ui.label(egui::RichText::new("No violations.").color(accent(ui)));
            return;
        }
        ui.label(match violations.len() {
            1 => "1 finding. Click it to zoom to it on the board.".to_owned(),
            n => format!("{n} findings. Click one to zoom to it on the board."),
        });
        let mut focus = None;
        let mut rows = vec![];
        ui.scope(|ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Wrap);
            for (i, v) in violations.iter().enumerate().take(200) {
                let row = finding_row(ui.style(), v);
                let focused = self.board_view.focused_violation == Some(i);
                let row = ui.selectable_label(focused, row).on_hover_text("Zoom to it on the board");
                if row.clicked() {
                    focus = Some((i, v.at));
                }
                rows.push(row);
            }
        });
        // Each finding by its place in the list, which is the order the check
        // returned them in.
        for (i, row) in rows.iter().enumerate() {
            self.board_view.inspector.mark(format!("row:finding:{i}"), row);
        }
        if let Some((i, at)) = focus {
            let extent = self.finding_extent(&self.board_view.violations.as_ref().unwrap()[i]);
            self.board_view.focused_violation = Some(i);
            self.board_view.focus_on(at, extent);
        }
    }
    /// How far a design rule finding reaches, in µm: an unrouted connection runs
    /// between its two pads, and is found again in the ratsnest by its net and its
    /// midpoint, which is where the check puts it. Every other finding is a point.
    fn finding_extent(&self, v: &Violation) -> i32 {
        if v.kind != ViolationKind::Unrouted {
            return 0;
        }
        let conn = self.document.board.connectivity(&self.document.netlist());
        self.document
            .board
            .ratsnest(&conn)
            .iter()
            .find(|w| {
                Point::new((w.a.x + w.b.x) / 2, (w.a.y + w.b.y) / 2) == v.at
                    && v.message.ends_with(&format!(" {}", w.net))
            })
            .map_or(0, |w| (w.b.x - w.a.x).abs().max((w.b.y - w.a.y).abs()))
    }
    fn outline_section(&mut self, ui: &mut egui::Ui) {
        let (min, max) = self.document.board.outline_bounds();
        let rectangle_outline = self.document.board.outline_is_rectangle();
        if rectangle_outline {
            let (w, h) = *self
                .board_view
                .outline
                .get_or_insert((mm(max.x - min.x), mm(max.y - min.y)));
            let (mut w, mut h) = (w, h);
            ui.horizontal(|ui| {
                ui.label("Width");
                let field = ui.add(
                    egui::DragValue::new(&mut w)
                        .speed(0.5)
                        .range(5.0..=1000.0)
                        .suffix(" mm"),
                );
                self.board_view.inspector.mark("outline_width", &field);
            });
            ui.horizontal(|ui| {
                ui.label("Height");
                let field = ui.add(
                    egui::DragValue::new(&mut h)
                        .speed(0.5)
                        .range(5.0..=1000.0)
                        .suffix(" mm"),
                );
                self.board_view.inspector.mark("outline_height", &field);
            });
            self.board_view.outline = Some((w, h));
            let changed = um(w) != max.x - min.x || um(h) != max.y - min.y;
            let apply = ui.add_enabled(changed, egui::Button::new("Apply size"));
            self.board_view.inspector.mark("apply_size", &apply);
            if apply.clicked() {
                self.transaction(|d| {
                    d.board.outline = rectangle(min, Point::new(min.x + um(w), min.y + um(h)))
                });
                self.board_view.outline = None;
            }
        } else {
            ui.small("Custom outline");
        }
        if let Some(note) = stock_board_note(&self.document.board) {
            ui.label(egui::RichText::new(note).color(ui.visuals().warn_fg_color));
        }
        // Both say what they did: the board's message stayed on the last sync's line
        // for ten steps of the re-audit's walk while these two ran in silence.
        let size = |board: &Board| {
            let (min, max) = board.outline_bounds();
            Point::new(max.x - min.x, max.y - min.y)
        };
        let fit = ui.button("Fit outline to parts");
        self.board_view.inspector.mark("fit_outline", &fit);
        if fit.clicked() {
            let from = size(&self.document.board);
            self.transaction(|d| d.fit_board_outline(FIT_MARGIN));
            self.board_view.outline = None;
            // The view follows the outline it was fitted to, as it does a sync's.
            self.board_view.fitted = false;
            let to = size(&self.document.board);
            self.board_view.message = if self.document.board.placements.is_empty() {
                "There are no parts to fit the outline to.".into()
            } else if from == to {
                format!("The outline already fits the parts: {} mm.", size_words(to))
            } else {
                format!(
                    "Outline fitted to the parts, {:.0} mm clear of them: {} mm, from {} mm.",
                    mm(FIT_MARGIN),
                    size_words(to),
                    size_words(from)
                )
            };
        }
        let place = ui
            .button("Auto-place parts")
            .on_hover_text("Lay the parts out afresh: the part with the most pins in the middle, and each other part as near as there is room to the parts it shares a net with. Sides and rotations are kept.");
        self.board_view.inspector.mark("auto_place", &place);
        if place.clicked() {
            let from = size(&self.document.board);
            self.transaction(Document::place_by_nets);
            let to = size(&self.document.board);
            let count = self.document.board.placements.len();
            self.board_view.message = match count {
                0 => "There are no parts to place.".into(),
                _ if from != to => format!(
                    "{count} parts placed by their nets. They did not fit, so the board grew from {} to {} mm.",
                    size_words(from),
                    size_words(to)
                ),
                _ => format!("{count} parts placed by their nets, each beside the parts it connects to."),
            };
            self.board_view.outline = None;
        }
        ui.horizontal(|ui| {
            ui.label("Copper layers");
            let mut count = self.document.board.layer_count;
            let combo = egui::ComboBox::from_id_salt("layer-count")
                .selected_text(count.to_string())
                .show_ui(ui, |ui| {
                    for n in [1u8, 2, 4, 6] {
                        let choice = ui.selectable_value(&mut count, n, n.to_string());
                        self.board_view.inspector.mark(format!("copper_layers:{n}"), &choice);
                    }
                });
            self.board_view.inspector.mark("copper_layers", &combo.response);
            if count != self.document.board.layer_count {
                let mut draft = self.document.board.clone();
                match draft.set_layer_count(count) {
                    Ok(()) => {
                        self.transaction(|d| d.board = draft);
                        self.board_view.active_layer = self.board_view.active_layer.min(count - 1);
                        self.board_view.hidden.clear();
                    }
                    Err(e) => self.board_view.message = e,
                }
            }
        });
    }
    fn rules_section(&mut self, ui: &mut egui::Ui) {
        let current = self.document.board.rules.clone();
        let mut rules = self
            .board_view
            .rules
            .take()
            .unwrap_or_else(|| current.clone());
        // The Net classes section edits the classes directly; a rules draft never
        // holds its own copy of them, so Apply cannot put back old ones.
        rules.net_classes = current.net_classes.clone();
        rules.net_class_of = current.net_class_of.clone();
        ui.label(
            egui::RichText::new("Track width, clearance and via are the Default net class.")
                .small()
                .color(ui.visuals().weak_text_color()),
        );
        for (label, value, max) in [
            ("Track width", &mut rules.track_width, 5.0),
            ("Clearance", &mut rules.clearance, 5.0),
            ("Via diameter", &mut rules.via_diameter, 5.0),
            ("Via drill", &mut rules.via_drill, 5.0),
            ("Edge clearance", &mut rules.edge_clearance, 10.0),
            ("Routing grid", &mut rules.routing_grid, 2.5),
        ] {
            ui.horizontal(|ui| {
                ui.label(label);
                let mut v = mm(*value);
                let field = ui.add(
                    egui::DragValue::new(&mut v)
                        .speed(0.005)
                        .range(0.025..=max)
                        .fixed_decimals(3)
                        .suffix(" mm"),
                );
                self.board_view.inspector.mark(format!("rule:{}", key_word(label)), &field);
                if field.changed() {
                    *value = um(v);
                }
            });
        }
        ui.horizontal(|ui| {
            ui.label("Solder mask expansion");
            let mut v = mm(rules.mask_expansion);
            let field = ui.add(
                egui::DragValue::new(&mut v)
                    .speed(0.005)
                    .range(0.0..=0.5)
                    .fixed_decimals(3)
                    .suffix(" mm"),
            );
            self.board_view.inspector.mark("rule:solder_mask_expansion", &field);
            if field.changed() {
                rules.mask_expansion = um(v);
            }
        });
        ui.label("Net widths");
        let mut remove = None;
        for (net, width) in &rules.net_widths {
            ui.horizontal(|ui| {
                ui.small(format!("{net}: {:.3} mm", mm(*width)));
                let remove_button = ui.small_button("Remove");
                self.board_view.inspector.mark(format!("net_width:{net}:remove"), &remove_button);
                if remove_button.clicked() {
                    remove = Some(net.clone());
                }
            });
        }
        if let Some(net) = remove {
            rules.net_widths.remove(&net);
        }
        let names: BTreeSet<String> = self
            .document
            .netlist()
            .nets
            .into_iter()
            .filter(|n| !n.labels.is_empty() && n.pins.len() > 1)
            .map(|n| n.name)
            .collect();
        ui.horizontal(|ui| {
            let combo = egui::ComboBox::from_id_salt("net-width-net")
                .selected_text(if self.board_view.net_width.0.is_empty() {
                    "Net…"
                } else {
                    self.board_view.net_width.0.as_str()
                })
                .show_ui(ui, |ui| {
                    menu_ui(ui);
                    for name in &names {
                        let choice =
                            ui.selectable_value(&mut self.board_view.net_width.0, name.clone(), name);
                        self.board_view.inspector.mark(format!("net_width_net:{name}"), &choice);
                    }
                });
            self.board_view.inspector.mark("net_width_net", &combo.response);
            let field = ui.add(
                egui::DragValue::new(&mut self.board_view.net_width.1)
                    .speed(0.01)
                    .range(0.05..=5.0)
                    .suffix(" mm"),
            );
            self.board_view.inspector.mark("net_width", &field);
            let add = ui.add_enabled(
                !self.board_view.net_width.0.is_empty(),
                egui::Button::new("Add"),
            );
            self.board_view.inspector.mark("add_net_width", &add);
            if add.clicked() {
                rules.net_widths.insert(
                    self.board_view.net_width.0.clone(),
                    um(self.board_view.net_width.1),
                );
            }
        });
        let valid = rules.via_drill < rules.via_diameter;
        if !valid {
            ui.small(
                egui::RichText::new("The via drill must be smaller than the via.")
                    .color(ui.visuals().error_fg_color),
            );
        }
        let apply = ui.add_enabled(valid && rules != current, egui::Button::new("Apply rules"));
        self.board_view.inspector.mark("apply_rules", &apply);
        if apply.clicked() {
            self.transaction(|d| d.board.rules = rules.clone());
            self.board_view.rules = None;
        } else if rules != current {
            self.board_view.rules = Some(rules);
        }
    }

    /// The net of what is selected on the board — a track's, a via's, a pad's or a
    /// zone's — if it has exactly one.
    pub fn selected_board_net(&self) -> Option<String> {
        let board = &self.document.board;
        match self.board_view.selected? {
            BoardSelection::Track(id) => self.document.track_nets().get(&id).cloned(),
            BoardSelection::Zone(id) => board.zones.iter().find(|z| z.id == id).map(|z| z.net.clone()),
            BoardSelection::Pad(component, index) => {
                let number = &board.placement(component)?.footprint.pads.get(index)?.number;
                self.document
                    .netlist()
                    .nets
                    .into_iter()
                    .find(|n| n.pins.iter().any(|p| p.component_id == component && &p.number == number))
                    .map(|n| n.name)
            }
            BoardSelection::Via(id) => {
                let conn = board.connectivity(&self.document.netlist());
                let i = conn.items.iter().position(|item| {
                    matches!(item.owner, CopperRef::Via(v) if board.vias.get(v).is_some_and(|via| via.id == id))
                })?;
                conn.island_net(conn.islands[i]).map(str::to_owned)
            }
            _ => None,
        }
    }
    /// The net the Net classes section is about: the selection's, else the one
    /// picked in the section.
    pub fn net_class_subject(&self) -> Option<String> {
        self.selected_board_net()
            .or_else(|| (!self.board_view.class_net.is_empty()).then(|| self.board_view.class_net.clone()))
    }
    /// Try `f` on a copy of the board's rules and keep it only if the classes are
    /// still valid; otherwise say why in the Inspector's message.
    fn edit_net_classes(&mut self, coalesce: Option<String>, f: impl FnOnce(&mut DesignRules)) -> Result<(), String> {
        let mut rules = self.document.board.rules.clone();
        f(&mut rules);
        rules.check_net_classes()?;
        if rules != self.document.board.rules {
            self.transaction_as(coalesce, |d| d.board.rules = rules);
        }
        Ok(())
    }
    /// Net classes: every class with its clearance, track width and via, the name
    /// patterns that put nets in it and the nets it holds; adding, editing and
    /// deleting one; and which class one net is in, why, and the class to put it in.
    fn net_classes_section(&mut self, ui: &mut egui::Ui) {
        let rules = self.document.board.rules.clone();
        let nets = self.net_names();
        let sub = |ui: &mut egui::Ui, text: String| {
            ui.label(egui::RichText::new(text).small().color(ui.visuals().weak_text_color()));
        };
        let size = |c: i32, w: i32, d: i32, h: i32| {
            format!("{:.3} clearance · {:.3} track · via {:.3}/{:.3} mm", mm(c), mm(w), mm(d), mm(h))
        };

        // One net: its class, why, and what it asks for; and a class to put it in.
        let subject = self.net_class_subject();
        ui.horizontal(|ui| {
            ui.label("Net");
            let combo = egui::ComboBox::from_id_salt("net-class-net")
                .selected_text(subject.as_deref().unwrap_or("Net…"))
                .show_ui(ui, |ui| {
                    menu_ui(ui);
                    for name in &nets {
                        let choice = ui.selectable_label(subject.as_ref() == Some(name), name);
                        self.board_view.inspector.mark(format!("net_class:net:{name}"), &choice);
                        if choice.clicked() {
                            self.board_view.class_net = name.clone();
                            self.board_view.selected = None;
                        }
                    }
                });
            self.board_view.inspector.mark("net_class:net", &combo.response);
        });
        if let Some(net) = &subject {
            let (class, reason) = rules.class_reason(Some(net));
            let why = match &reason {
                ClassReason::Assigned => "assigned".to_owned(),
                ClassReason::Pattern(p) => format!("by the pattern {p}"),
                ClassReason::Default => "nothing puts it in another class".to_owned(),
            };
            let readout = ui.label(
                egui::RichText::new(format!("{net} is {} — {why}", class.name)).color(accent(ui)),
            );
            self.board_view.inspector.mark("net_class:readout", &readout);
            let width = rules.width_for(net);
            sub(
                ui,
                format!(
                    "{}{}",
                    size(class.clearance, width, class.via_diameter, class.via_drill),
                    if rules.net_widths.contains_key(net) { " (its own track width)" } else { "" }
                ),
            );
            let assigned = rules.net_class_of.get(net).filter(|_| reason == ClassReason::Assigned).cloned();
            let mut choice = assigned.clone();
            ui.horizontal(|ui| {
                ui.label("Put in");
                let combo = egui::ComboBox::from_id_salt("net-class-assign")
                    .selected_text(choice.clone().unwrap_or_else(|| "by pattern".into()))
                    .show_ui(ui, |ui| {
                        menu_ui(ui);
                        let auto = ui.selectable_value(&mut choice, None, "by pattern")
                            .on_hover_text("No assignment: the first class whose pattern matches, else Default.");
                        self.board_view.inspector.mark("net_class:assign:auto", &auto);
                        for c in &rules.net_classes {
                            let option = ui.selectable_value(&mut choice, Some(c.name.clone()), &c.name);
                            self.board_view.inspector.mark(format!("net_class:assign:{}", c.name), &option);
                        }
                    });
                self.board_view.inspector.mark("net_class:assign", &combo.response);
            });
            if choice != assigned {
                let net = net.clone();
                let _ = self.edit_net_classes(None, |r| match &choice {
                    Some(class) => {
                        r.net_class_of.insert(net.clone(), class.clone());
                    }
                    None => {
                        r.net_class_of.remove(&net);
                    }
                });
                self.board_view.message = match &choice {
                    Some(class) => format!("{net} is in {class}. Run the design rule check to see what it changes."),
                    None => format!("{net} follows the patterns: {}.", self.document.board.rules.class_of(Some(&net)).name),
                };
            }
        } else {
            sub(ui, "Select a track, pad, via or zone, or pick a net, to see its class.".into());
        }
        ui.add_space(6.);

        // Which class each net is in, for the lists under each class.
        let mut members: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for net in &nets {
            members.entry(rules.class_of(Some(net)).name).or_default().push(net.clone());
        }
        let listed = |names: Option<&Vec<String>>| match names {
            None => "no nets".to_owned(),
            Some(names) if names.len() > 8 => format!("{} nets: {}, …", names.len(), names[..8].join(", ")),
            Some(names) => format!("{} net{}: {}", names.len(), if names.len() == 1 { "" } else { "s" }, names.join(", ")),
        };

        // Default: the board's own rules, edited under Design rules.
        let default = rules.default_class();
        let row = ui.label(egui::RichText::new(DEFAULT_CLASS).strong());
        self.board_view.inspector.mark(format!("net_class:{DEFAULT_CLASS}"), &row);
        sub(ui, size(default.clearance, default.track_width, default.via_diameter, default.via_drill));
        sub(ui, format!("{} — its sizes are the Design rules above", listed(members.get(DEFAULT_CLASS))));

        let mut delete = None;
        for (i, class) in rules.net_classes.iter().enumerate() {
            ui.separator();
            let key = class.name.clone();
            ui.horizontal(|ui| {
                let (mut text, _) = self
                    .board_view
                    .class_text
                    .get(&(i, "name"))
                    .cloned()
                    .unwrap_or_else(|| (class.name.clone(), String::new()));
                let field = ui.add(egui::TextEdit::singleline(&mut text).id_salt(("net-class-name", i)).desired_width(110.));
                self.board_view.inspector.mark(format!("net_class:{key}:name"), &field);
                if field.changed() {
                    let old = class.name.clone();
                    let new = text.clone();
                    let result = self.edit_net_classes(Some(format!("net_class:{i}:name")), |r| {
                        r.net_classes[i].name = new.clone();
                        for assigned in r.net_class_of.values_mut() {
                            if *assigned == old {
                                *assigned = new.clone();
                            }
                        }
                    });
                    match result {
                        Ok(()) => {
                            self.board_view.class_text.remove(&(i, "name"));
                        }
                        Err(e) => {
                            self.board_view.class_text.insert((i, "name"), (text, e));
                        }
                    }
                } else if field.lost_focus() {
                    self.board_view.class_text.remove(&(i, "name"));
                }
                let remove = ui.small_button("Delete").on_hover_text("Its nets go back to the patterns, else Default.");
                self.board_view.inspector.mark(format!("net_class:{key}:delete"), &remove);
                if remove.clicked() {
                    delete = Some(i);
                }
            });
            if let Some((_, why)) = self.board_view.class_text.get(&(i, "name")) {
                ui.small(egui::RichText::new(why).color(ui.visuals().error_fg_color));
            }
            for (field_key, label, get, max) in [
                ("clearance", "Clearance", class.clearance, 5.0),
                ("track_width", "Track width", class.track_width, 5.0),
                ("via_diameter", "Via diameter", class.via_diameter, 5.0),
                ("via_drill", "Via drill", class.via_drill, 5.0),
            ] {
                ui.horizontal(|ui| {
                    ui.label(label);
                    let mut v = mm(get);
                    let field = ui.add(
                        egui::DragValue::new(&mut v)
                            .speed(0.005)
                            .range(if field_key == "clearance" { 0.0 } else { 0.025 }..=max)
                            .fixed_decimals(3)
                            .suffix(" mm"),
                    );
                    self.board_view.inspector.mark(format!("net_class:{key}:{field_key}"), &field);
                    if field.changed() {
                        let result = self.edit_net_classes(Some(format!("net_class:{i}:{field_key}")), |r| {
                            let c = &mut r.net_classes[i];
                            let value = um(v);
                            match field_key {
                                "clearance" => c.clearance = value,
                                "track_width" => c.track_width = value,
                                "via_diameter" => c.via_diameter = value,
                                _ => c.via_drill = value,
                            }
                        });
                        if let Err(e) = result {
                            self.board_view.message = format!("{e}: the via drill must be smaller than the via.");
                        }
                    }
                });
            }
            ui.horizontal(|ui| {
                ui.label("Patterns");
                let (mut text, _) = self
                    .board_view
                    .class_text
                    .get(&(i, "patterns"))
                    .cloned()
                    .unwrap_or_else(|| (class.patterns.join(", "), String::new()));
                let field = ui
                    .add(egui::TextEdit::singleline(&mut text).id_salt(("net-class-patterns", i)).hint_text("+*V*, GND"))
                    .on_hover_text("Net names this class takes, separated by commas: * is any run of characters, ? any one. An assignment beats a pattern; the first class whose pattern matches wins.");
                self.board_view.inspector.mark(format!("net_class:{key}:patterns"), &field);
                if field.changed() {
                    let patterns: Vec<String> =
                        text.split(',').map(str::trim).filter(|p| !p.is_empty()).map(str::to_owned).collect();
                    let _ = self.edit_net_classes(Some(format!("net_class:{i}:patterns")), |r| {
                        r.net_classes[i].patterns = patterns;
                    });
                    self.board_view.class_text.insert((i, "patterns"), (text, String::new()));
                } else if field.lost_focus() {
                    self.board_view.class_text.remove(&(i, "patterns"));
                }
            });
            sub(ui, listed(members.get(&class.name)));
        }
        if let Some(i) = delete {
            let name = rules.net_classes[i].name.clone();
            let _ = self.edit_net_classes(None, |r| {
                r.net_classes.remove(i);
                r.net_class_of.retain(|_, class| *class != name);
            });
            self.board_view.class_text.clear();
            self.board_view.message = format!("Deleted net class {name}: its nets follow the patterns, else Default.");
        }
        ui.add_space(4.);
        let add = ui.button("Add net class").on_hover_text("A new class with the Default class's sizes.");
        self.board_view.inspector.mark("net_class:add", &add);
        if add.clicked() {
            let taken: BTreeSet<String> = rules.classes().into_iter().map(|c| c.name).collect();
            let name = (1..).map(|n| format!("Class {n}")).find(|n| !taken.contains(n)).unwrap_or_default();
            let class = NetClass { name: name.clone(), ..rules.default_class() };
            let _ = self.edit_net_classes(None, |r| r.net_classes.push(class));
            self.board_view.message = format!("Added net class {name}: give it patterns, or put a net in it above.");
        }
    }

    fn switch_layer(&mut self, layer: u8) {
        let count = self.document.board.layer_count;
        let layer = layer.min(count - 1);
        // The via is the size its net's class asks for, as the track's width is.
        let (diameter, drill) = {
            let net = self.board_view.draft.as_ref().and_then(|draft| self.draft_net(draft));
            self.document.board.rules.via_for(net.as_deref())
        };
        if let Some(draft) = &mut self.board_view.draft
            && draft.layer != layer
        {
            let at = *draft.points.last().unwrap();
            let points = std::mem::replace(&mut draft.points, vec![at]);
            let track = Track::new(draft.layer, draft.width, points);
            if track.points.len() >= 2 {
                draft.tracks.push(track);
            }
            draft.vias.push(Via {
                id: Uuid::new_v4(),
                at,
                diameter,
                drill,
            });
            draft.layer = layer;
        }
        self.board_view.active_layer = layer;
    }
    fn finish_draft(&mut self) {
        let Some(mut draft) = self.board_view.draft.take() else {
            return;
        };
        let track = Track::new(draft.layer, draft.width, draft.points);
        if track.points.len() >= 2 {
            draft.tracks.push(track);
        }
        if draft.tracks.is_empty() && draft.vias.is_empty() {
            return;
        }
        self.transaction(|d| {
            d.board.tracks.extend(draft.tracks);
            d.board.vias.extend(draft.vias);
        });
    }
    /// Snap a routing click to pads, vias, and tracks, or else to the placement grid.
    fn route_target(&self, p: Pos2) -> (Point, Option<Anchor>) {
        let view = &self.board_view;
        let board = &self.document.board;
        let q = view.world(p);
        let tolerance = view.tolerance();
        for placement in board.placements.iter().rev() {
            for pad in &placement.footprint.pads {
                let layers = placement.pad_layers(pad, board.layer_count);
                if (layers.0..=layers.1).any(|l| view.visible(l))
                    && placement.pad_shape(pad).distance_to_point(q) <= tolerance
                {
                    let width = self.pad_width(placement.component, &pad.number);
                    return (
                        placement.transform(pad.at),
                        Some(Anchor::Pad { layers, width }),
                    );
                }
            }
        }
        for via in &board.vias {
            if Shape::circle(via.at, via.diameter / 2).distance_to_point(q) <= tolerance {
                return (via.at, Some(Anchor::Via));
            }
        }
        for track in board.tracks.iter().filter(|t| view.visible(t.layer)) {
            for s in track.points.windows(2) {
                if Shape::segment(s[0], s[1], track.width).distance_to_point(q) <= tolerance {
                    let (a, b) = (s[0], s[1]);
                    let (dx, dy) = (f64::from(b.x - a.x), f64::from(b.y - a.y));
                    let len2 = (dx * dx + dy * dy).max(1.);
                    let t = ((f64::from(q.x - a.x) * dx + f64::from(q.y - a.y) * dy) / len2)
                        .clamp(0., 1.);
                    let on =
                        Point::new(a.x + (t * dx).round() as i32, a.y + (t * dy).round() as i32);
                    let at = [a, b]
                        .into_iter()
                        .find(|v| f64::from(v.x - on.x).hypot(f64::from(v.y - on.y)) <= tolerance)
                        .unwrap_or(on);
                    return (
                        at,
                        Some(Anchor::Track {
                            layer: track.layer,
                            width: track.width,
                        }),
                    );
                }
            }
        }
        (snap(q, PLACE_GRID), None)
    }
    fn route_click(&mut self, p: Pos2) {
        let (at, anchor) = self.route_target(p);
        let Some(draft) = &self.board_view.draft else {
            self.start_draft(at, anchor);
            return;
        };
        if at == *draft.points.last().unwrap() {
            return;
        }
        // The click places exactly the leg the preview drew: the same corner, and
        // the same verdict on its clearance.
        let conn = self.document.board.connectivity(&self.document.netlist());
        let leg = self.draft_leg(&conn, draft, at, anchor.is_some());
        let message = match (&leg.clash, &leg.avoided) {
            (Some(clash), _) => Some(format!(
                "{}: this track passes {}{}. No corner keeps it clear, so it is drawn as you asked; Undo takes it back.",
                if clash.crosses() { "Short" } else { "Clearance" },
                clash.words(),
                if clash.crosses() {
                    String::new()
                } else {
                    format!(", and the board needs {:.3} mm ({} clearance)", f64::from(clash.needed) / 1000., clash.class)
                },
            )),
            (None, Some(avoided)) => Some(format!(
                "Bent the other way: the corner you asked for passed {}.",
                avoided.words()
            )),
            (None, None) => None,
        };
        if let Some(draft) = &mut self.board_view.draft {
            draft.points.extend(leg.points.into_iter().skip(1));
        }
        if anchor.is_some() {
            self.finish_draft();
        }
        if let Some(message) = message {
            self.board_view.message = message;
        }
    }
    /// The one net the copper the draft began on carries, if it carries exactly one.
    fn draft_net(&self, draft: &Draft) -> Option<String> {
        let origin = Self::draft_origin(draft);
        let conn = self.document.board.connectivity(&self.document.netlist());
        let nets: BTreeSet<&String> = conn
            .items
            .iter()
            .enumerate()
            .filter(|(_, item)| item.shape.distance_to_point(origin) <= 0.5)
            .flat_map(|(i, _)| &conn.island_nets[conn.islands[i]])
            .collect();
        let mut nets = nets.into_iter();
        match (nets.next(), nets.next()) {
            (Some(net), None) => Some(net.clone()),
            _ => None,
        }
    }
    /// The first point of the track being drawn, on whichever layer it began.
    fn draft_origin(draft: &Draft) -> Point {
        draft
            .tracks
            .first()
            .and_then(|t| t.points.first())
            .or(draft.points.first())
            .copied()
            .unwrap_or_default()
    }
    /// The closest copper of another net that `points`, drawn at `width` on
    /// `layer`, comes within the board's clearance of — the check DRC makes,
    /// made while the track is still a draft. Copper is the track's OWN when it
    /// touches the track's first point, or carries a net that copper does; and
    /// when the track ends on copper (`end`), that copper is its own too unless
    /// it carries only other nets, which would be a short, and is reported.
    fn route_clash(
        &self,
        conn: &Connectivity,
        (layer, width): (u8, i32),
        origin: Point,
        end: Option<Point>,
        points: &[Point],
    ) -> Option<Clash> {
        let touching = |p: Point| -> BTreeSet<usize> {
            conn.items
                .iter()
                .enumerate()
                .filter(|(_, item)| item.shape.distance_to_point(p) <= 0.5)
                .map(|(i, _)| conn.islands[i])
                .collect()
        };
        let mut own = touching(origin);
        let nets: BTreeSet<&String> = own.iter().flat_map(|&i| &conn.island_nets[i]).collect();
        for (island, carried) in conn.island_nets.iter().enumerate() {
            if carried.iter().any(|net| nets.contains(net)) {
                own.insert(island);
            }
        }
        if let Some(end) = end {
            for island in touching(end) {
                if nets.is_empty() || conn.island_nets[island].is_empty() {
                    own.insert(island);
                }
            }
        }
        let rules = &self.document.board.rules;
        let mut worst: Option<(f64, usize, i32, String)> = None;
        for leg in points.windows(2).filter(|leg| leg[0] != leg[1]) {
            let copper = Shape::segment(leg[0], leg[1], width);
            for (i, item) in conn.items.iter().enumerate() {
                if own.contains(&conn.islands[i]) || !(item.layers.0..=item.layers.1).contains(&layer) {
                    continue;
                }
                let (needed, class) = rules.clearance_between_nets(
                    nets.iter().map(|n| n.as_str()),
                    conn.island_nets[conn.islands[i]].iter().map(String::as_str),
                );
                let gap = copper.distance(&item.shape);
                if gap + 0.5 < f64::from(needed) && worst.as_ref().is_none_or(|w| gap < w.0) {
                    worst = Some((gap, i, needed, class.name));
                }
            }
        }
        let (gap, i, needed, class) = worst?;
        let item = &conn.items[i];
        let board = &self.document.board;
        let what = match item.owner {
            CopperRef::Pad { placement, pad } => board.placements.get(placement).map_or_else(
                || "a pad".to_owned(),
                |placement| {
                    let number = placement.footprint.pads.get(pad).map_or("", |p| p.number.as_str());
                    format!("{} pad {number}", self.reference_of(placement.component))
                },
            ),
            CopperRef::Track { track, .. } => board.tracks.get(track).map_or_else(
                || "a track".to_owned(),
                |t| format!("a track on {}", layer_name(t.layer, board.layer_count)),
            ),
            CopperRef::Via(_) => "a via".to_owned(),
        };
        let nets = &conn.island_nets[conn.islands[i]];
        let net = (!nets.is_empty()).then(|| nets.iter().cloned().collect::<Vec<_>>().join(", "));
        Some(Clash { gap: gap.max(0.), what, net, at: item.shape.center(), needed, class })
    }
    /// What part `id`, where it is now, would come down on: its pads against the
    /// copper of the board as it was when the drag began (`before`, whose islands
    /// carry the nets that copper was on, and which holds the part's own pads where
    /// they were), the test DRC makes, made while the part is still moving. A pad
    /// is clear of copper on its own net; one that touches copper of another net
    /// is a short; and one that joins copper that carries no net yet, or carries
    /// none itself, joins it, as DRC will read it. Courtyards are compared on the
    /// part's own side, as DRC compares them.
    fn landfall(&self, id: Uuid, before: &Connectivity) -> Landfall {
        let board = &self.document.board;
        let Some(index) = board.placements.iter().position(|p| p.component == id) else {
            return Landfall::default();
        };
        let placement = &board.placements[index];
        let reference = self.reference_of(id);
        let mut pad_nets = BTreeMap::new();
        for (i, item) in before.items.iter().enumerate() {
            if let CopperRef::Pad { placement, pad } = item.owner
                && placement == index
            {
                pad_nets.insert(pad, before.item_nets[i].clone());
            }
        }
        let mut copper = vec![];
        for (k, pad) in placement.footprint.pads.iter().enumerate() {
            let shape = placement.pad_shape(pad);
            let layers = placement.pad_layers(pad, board.layer_count);
            let net = pad_nets.get(&k).cloned().flatten();
            let mut worst: Option<(f64, usize, i32, String)> = None;
            for (i, item) in before.items.iter().enumerate() {
                if matches!(item.owner, CopperRef::Pad { placement, .. } if placement == index)
                    || layers.0 > item.layers.1
                    || item.layers.0 > layers.1
                {
                    continue;
                }
                let theirs = &before.island_nets[before.islands[i]];
                if net.as_ref().is_some_and(|net| theirs.contains(net)) {
                    continue;
                }
                let (needed, class) =
                    board.rules.clearance_between_nets(net.as_deref(), theirs.iter().map(String::as_str));
                let gap = shape.distance(&item.shape);
                let joins = gap <= 0.5 && (net.is_none() || theirs.is_empty());
                if gap + 0.5 < f64::from(needed) && !joins && worst.as_ref().is_none_or(|w| gap < w.0) {
                    worst = Some((gap, i, needed, class.name));
                }
            }
            if let Some((gap, i, needed, class)) = worst {
                let item = &before.items[i];
                let what = match item.owner {
                    CopperRef::Pad { placement, pad } => board.placements.get(placement).map_or_else(
                        || "a pad".to_owned(),
                        |other| {
                            let number = other.footprint.pads.get(pad).map_or("", |p| p.number.as_str());
                            format!("{} pad {number}", self.reference_of(other.component))
                        },
                    ),
                    CopperRef::Track { track, .. } => board.tracks.get(track).map_or_else(
                        || "a track".to_owned(),
                        |t| format!("a track on {}", layer_name(t.layer, board.layer_count)),
                    ),
                    CopperRef::Via(_) => "a via".to_owned(),
                };
                let nets = &before.island_nets[before.islands[i]];
                let on = (!nets.is_empty()).then(|| nets.iter().cloned().collect::<Vec<_>>().join(", "));
                let mine = if pad.number.is_empty() {
                    format!("a mechanical pad of {reference}")
                } else {
                    format!("{reference} pad {}", pad.number)
                };
                copper.push((mine, Clash { gap: gap.max(0.), what, net: on, at: item.shape.center(), needed, class }));
            }
        }
        copper.sort_by(|a, b| a.1.gap.total_cmp(&b.1.gap));
        let (a0, a1) = placement.courtyard();
        let courtyards = board
            .placements
            .iter()
            .enumerate()
            .filter(|(j, other)| *j != index && other.bottom == placement.bottom)
            .filter(|(_, other)| {
                let (b0, b1) = other.courtyard();
                a0.x < b1.x && b0.x < a1.x && a0.y < b1.y && b0.y < a1.y
            })
            .map(|(_, other)| (self.reference_of(other.component).to_owned(), other.at))
            .collect();
        Landfall { copper, courtyards }
    }
    /// The next leg of the draft, from its last point to `target`: the corner the
    /// user chose, or the other one when the chosen one comes within the
    /// clearance of another net's copper and the other does not. A leg that
    /// clears neither way is drawn as chosen, with its clash.
    fn draft_leg(&self, conn: &Connectivity, draft: &Draft, target: Point, ends_on_copper: bool) -> Leg {
        let last = *draft.points.last().unwrap();
        let origin = Self::draft_origin(draft);
        let end = ends_on_copper.then_some(target);
        let chosen = self.board_view.diagonal_first;
        let try_corner = |diagonal_first: bool| {
            let points = corner(last, target, diagonal_first);
            let clash = self.route_clash(conn, (draft.layer, draft.width), origin, end, &points);
            (points, clash)
        };
        let (points, clash) = try_corner(chosen);
        if let Some(avoided) = clash.clone() {
            let (other, other_clash) = try_corner(!chosen);
            if other != points && other_clash.is_none() {
                return Leg { points: other, clash: None, avoided: Some(avoided) };
            }
        }
        Leg { points, clash, avoided: None }
    }
    /// Begin a track at `at`: on a pad's layer (the active one when the pad reaches
    /// it) at its net's width, on a track's layer at its width, or else on the active
    /// layer at the board's default width.
    fn start_draft(&mut self, at: Point, anchor: Option<Anchor>) {
        let active = self.board_view.active_layer;
        let (layer, width) = match anchor {
            Some(Anchor::Pad { layers, width }) => (
                if (layers.0..=layers.1).contains(&active) {
                    active
                } else {
                    layers.0
                },
                width,
            ),
            Some(Anchor::Track { layer, width }) => (layer, width),
            _ => (active, self.document.board.rules.track_width),
        };
        self.board_view.active_layer = layer;
        self.board_view.draft = Some(Draft {
            layer,
            width,
            points: vec![at],
            tracks: vec![],
            vias: vec![],
        });
    }
    /// The board's clickable things for [`Editor::hits`], in the board's own
    /// canvas: each part's courtyard, its numbered pads and its silkscreen
    /// lines, each track's longest segment's middle and each of its interior
    /// corners, each via.
    pub(crate) fn board_hits(&self) -> Vec<(String, Rect)> {
        let view = &self.board_view;
        let board = &self.document.board;
        let (_, zoom) = view.pan_zoom();
        let mut out = Vec::new();
        for p in &board.placements {
            let reference = self
                .document
                .components
                .iter()
                .find(|c| c.id == p.component)
                .map_or_else(|| p.component.to_string(), |c| c.reference.clone());
            let (min, max) = p.courtyard();
            out.push((
                format!("part:{reference}"),
                Rect::from_two_pos(view.screen(min), view.screen(max)),
            ));
            // A pad is keyed by its part and its number, as a sheet pin is
            // (`pin:R1.2`). A MECHANICAL pad has no number to be keyed by and
            // publishes none; it is clicked by its place on the canvas.
            for pad in p.footprint.pads.iter().filter(|pad| !pad.number.is_empty()) {
                let (min, max) = p.pad_shape(pad).bounds();
                let rect = Rect::from_two_pos(view.screen(min), view.screen(max));
                out.push((
                    format!("pad:{reference}.{}", pad.number),
                    Rect::from_center_size(rect.center(), rect.size().max(Vec2::splat(8.))),
                ));
            }
            // A silkscreen line is keyed by its part and its index, as the Pads
            // editor keys it by its index alone (`silk:0`): a small rect on the
            // middle of its longest segment. When a pad, a track or another line
            // covers that middle, the next longest segment whose middle the canvas
            // DOES answer with this line is used instead, so a click on the rect's
            // centre always selects what the key names.
            for (index, line) in p.footprint.silk.iter().enumerate() {
                let mut middles: Vec<(f32, Pos2)> = line
                    .windows(2)
                    .map(|s| {
                        (
                            view.screen(p.transform(s[0])),
                            view.screen(p.transform(s[1])),
                        )
                    })
                    .map(|(a, b)| (a.distance(b), a.lerp(b, 0.5)))
                    .collect();
                middles.sort_by(|a, b| b.0.total_cmp(&a.0));
                let Some(&(_, longest)) = middles.first() else {
                    continue;
                };
                let at = middles
                    .iter()
                    .map(|m| m.1)
                    .find(|&m| self.hit_board(m) == Some(BoardSelection::Silk(p.component, index)))
                    .unwrap_or(longest);
                out.push((
                    format!("silk:{reference}.{index}"),
                    Rect::from_center_size(at, Vec2::splat(8.)),
                ));
            }
        }
        for (i, t) in board.tracks.iter().enumerate() {
            let longest = t
                .points
                .windows(2)
                .map(|s| (view.screen(s[0]), view.screen(s[1])))
                .max_by(|a, b| a.0.distance(a.1).total_cmp(&b.0.distance(b.1)));
            if let Some((a, b)) = longest {
                out.push((
                    format!("track:{i}"),
                    Rect::from_center_size(a.lerp(b, 0.5), Vec2::splat(8.)),
                ));
            }
            // Each corner a drag can take, keyed by its point's index in the track.
            for j in 1..t.points.len().saturating_sub(1) {
                out.push((
                    format!("track:{i}.corner:{j}"),
                    Rect::from_center_size(view.screen(t.points[j]), Vec2::splat(8.)),
                ));
            }
        }
        for (i, v) in board.vias.iter().enumerate() {
            let size = (v.diameter as f32 * zoom).max(8.);
            out.push((
                format!("via:{i}"),
                Rect::from_center_size(view.screen(v.at), Vec2::splat(size)),
            ));
        }
        // A zone by its index: a small rect a quarter of the way along its longest
        // edge (the middle is the selected zone's new-corner handle), each corner
        // by its index, and, on the selected zone, each edge's middle.
        for (i, zone) in board.zones.iter().enumerate() {
            let n = zone.outline.len();
            let longest = (0..n).max_by_key(|&k| {
                let (a, b) = (zone.outline[k], zone.outline[(k + 1) % n]);
                i64::from(a.x - b.x).pow(2) + i64::from(a.y - b.y).pow(2)
            });
            if let Some(k) = longest {
                let (a, b) = (view.screen(zone.outline[k]), view.screen(zone.outline[(k + 1) % n]));
                out.push((format!("zone:{i}"), Rect::from_center_size(a.lerp(b, 0.25), Vec2::splat(8.))));
            }
            for (j, &c) in zone.outline.iter().enumerate() {
                out.push((format!("zone:{i}.corner:{j}"), Rect::from_center_size(view.screen(c), Vec2::splat(8.))));
            }
            if view.selected == Some(BoardSelection::Zone(zone.id)) {
                for (j, m) in edge_middles(&zone.outline).into_iter().enumerate() {
                    out.push((format!("zone:{i}.edge:{j}"), Rect::from_center_size(view.screen(m), Vec2::splat(8.))));
                }
            }
        }
        out.extend(view.menu.iter().cloned());
        out.extend(view.inspector.shown(view.canvas_pass));
        out
    }
    /// Whether a Select-tool press at `p` would start a drag.
    fn grabbable(&self, p: Pos2) -> bool {
        match self.hit_board(p) {
            Some(BoardSelection::Track(_)) => self.grip_at(p).is_some(),
            other => other.is_some(),
        }
    }
    /// What a press at `p` would grab of the track it hits, if it hits one.
    fn grip_at(&self, p: Pos2) -> Option<Grip> {
        let board = &self.document.board;
        let Some(BoardSelection::Track(id)) = self.hit_board(p) else {
            return None;
        };
        let track = board.tracks.iter().find(|t| t.id == id)?;
        grip(
            board,
            track,
            self.board_view.world(p),
            self.board_view.tolerance(),
        )
    }
    fn hit_board(&self, p: Pos2) -> Option<BoardSelection> {
        let view = &self.board_view;
        let board = &self.document.board;
        let q = view.world(p);
        let tolerance = view.tolerance();
        // The SELECTED track's own handles come first, before vias and pads. A
        // corner a route bent beside a via or a pad is drawn over it, and with
        // either first that handle could be seen and never taken. Only its
        // handles — a corner, or a free end — and only once the track is
        // selected: with nothing selected the via or pad still wins, and a press
        // on a segment's body still reaches whatever is under it. So does a press
        // on a HELD end, which `grip` never offers: an end resting on a via is
        // joined to it, and dragging the via is how that join is kept.
        if let Some(BoardSelection::Track(id)) = view.selected
            && let Some(track) = board.tracks.iter().find(|t| t.id == id)
            && view.visible(track.layer)
            && let Some(Grip::Vertex(_)) = grip(board, track, q, tolerance)
        {
            return Some(BoardSelection::Track(id));
        }
        // The SELECTED zone's corners and edge middles too, for the same reason:
        // a corner put down on a pad would otherwise be the pad's.
        if let Some(BoardSelection::Zone(id)) = view.selected
            && let Some(zone) = board.zones.iter().find(|z| z.id == id)
            && view.visible(zone.layer)
            && zone
                .outline
                .iter()
                .chain(&edge_middles(&zone.outline))
                .any(|&c| view.screen(c).distance(p) <= 7.)
        {
            return Some(BoardSelection::Zone(id));
        }
        if let Some(v) = board
            .vias
            .iter()
            .rev()
            .find(|v| Shape::circle(v.at, v.diameter / 2).distance_to_point(q) <= tolerance)
        {
            return Some(BoardSelection::Via(v.id));
        }
        // Pads come before TRACKS, because a routed pad wears a track's end in
        // the middle of it: with tracks first, the pad a user is aiming at is
        // exactly the part of it that is covered. A track is still selected
        // anywhere along its run.
        //
        // A pad is picked when the pointer is ON it, with NO slack at all — not
        // the tolerance the rest of this uses, and not the generous one ROUTING
        // snaps with. Slack only buys anything while a pad is small on screen,
        // and that is exactly when it would swallow the space between a part's
        // pads and leave no way to select the part itself. A pad too small to aim
        // at is reached from the part's own pad list instead.
        for placement in board.placements.iter().rev() {
            for (index, pad) in placement.footprint.pads.iter().enumerate() {
                let layers = placement.pad_layers(pad, board.layer_count);
                if (layers.0..=layers.1).any(|l| view.visible(l))
                    && placement.pad_shape(pad).contains(q)
                {
                    return Some(BoardSelection::Pad(placement.component, index));
                }
            }
        }
        let mut tracks: Vec<&Track> = board
            .tracks
            .iter()
            .filter(|t| view.visible(t.layer))
            .collect();
        tracks.sort_by_key(|t| t.layer != view.active_layer);
        if let Some(t) = tracks
            .into_iter()
            .find(|t| t.segments().any(|s| s.distance_to_point(q) <= tolerance))
        {
            return Some(BoardSelection::Track(t.id));
        }
        // SILKSCREEN after copper and before the courtyard: a line is legend
        // printed over the part, so copper under it is what a user is aiming at
        // when the two cross, and the courtyard around it is what they get when
        // they miss the line. A line is caught within the pointer tolerance of
        // any of its segments, the topmost part's first, as it is drawn.
        for placement in board.placements.iter().rev() {
            for (index, line) in placement.footprint.silk.iter().enumerate() {
                let near = line.windows(2).any(|s| {
                    let (a, b) = (placement.transform(s[0]), placement.transform(s[1]));
                    Shape::segment(a, b, 0).distance_to_point(q) <= tolerance
                });
                if near {
                    return Some(BoardSelection::Silk(placement.component, index));
                }
            }
        }
        // A zone's OUTLINE, after all the copper and legend that sit on it and
        // before the courtyards it runs through: the active layer's zones first.
        let mut zones: Vec<&Zone> = board.zones.iter().filter(|z| view.visible(z.layer)).collect();
        zones.sort_by_key(|z| z.layer != view.active_layer);
        let n = |z: &Zone| z.outline.len();
        if let Some(z) = zones.iter().find(|z| {
            (0..n(z)).any(|i| {
                Shape::segment(z.outline[i], z.outline[(i + 1) % n(z)], 0).distance_to_point(q) <= tolerance
            })
        }) {
            return Some(BoardSelection::Zone(z.id));
        }
        let bottom_first = view.active_layer + 1 == board.layer_count && board.layer_count > 1;
        let part = board
            .placements
            .iter()
            .filter(|p| {
                let (min, max) = p.courtyard();
                q.x >= min.x && q.x <= max.x && q.y >= min.y && q.y <= max.y
            })
            .min_by_key(|p| {
                let (min, max) = p.courtyard();
                (
                    p.bottom != bottom_first,
                    i64::from(max.x - min.x) * i64::from(max.y - min.y),
                )
            })
            .map(|p| BoardSelection::Part(p.component));
        // Inside a zone, last: a zone covers most of a board, and everything on
        // it comes first.
        part.or_else(|| {
            zones
                .iter()
                .find(|z| z.fill.iter().any(|piece| piece.contains(q)) || brep_ecad_core::board::polygon_contains(&z.outline, q))
                .map(|z| BoardSelection::Zone(z.id))
        })
    }

    /// Draw and interact with the board inside any egui Ui.
    pub(crate) fn show_board(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        self.board_view.canvas = rect;
        self.board_view.canvas_pass = ui.ctx().cumulative_pass_nr();
        self.board_view.menu.clear();
        self.canvas_size = rect.size();
        if !self.board_view.fitted {
            self.board_view.fit(&self.document.board);
        }
        self.seed_fill_baseline();
        let ctx = ui.ctx().clone();
        let pointer = ui
            .input(|i| i.pointer.hover_pos())
            .filter(|p| rect.contains(*p));
        if !self.routing() {
            self.board_input(ui, &response, pointer);
        }
        let netlist = self.document.netlist();
        let conn = self.document.board.connectivity(&netlist);
        let airwires = self.document.board.ratsnest(&conn);
        self.board_view.unrouted = airwires.len();
        // The leg a click would add now, judged for clearance while it is only a
        // preview: the click places this very leg ([`Self::route_click`]).
        let leg = match (&self.board_view.draft, pointer) {
            (Some(draft), Some(p)) => {
                let (target, anchor) = self.route_target(p);
                Some(self.draft_leg(&conn, draft, target, anchor.is_some()))
            }
            _ => None,
        };
        self.paint_board(&painter, &airwires, pointer, leg.as_ref());
        if let Some(p) = pointer {
            let q = self.board_view.world(p);
            // Say which pad is under the pointer BEFORE a click commits to it,
            // with the net it is on: a board is mostly small targets, and the
            // reference silkscreened over a part does not name its pads.
            let under = match self.hit_board(p) {
                Some(BoardSelection::Pad(id, index)) => {
                    self.pad_at(id, index)
                        .map(|(reference, pad)| {
                            // This frame's netlist, not another derivation of it: the
                            // pointer moves sixty times a second and the sheet has not.
                            let net = netlist.nets.iter().find(|net| {
                                !pad.number.is_empty()
                                    && net.pins.iter().any(|pin| {
                                        pin.component_id == id && pin.number == pad.number
                                    })
                            });
                            match (net, pad.number.is_empty()) {
                                (Some(net), _) => {
                                    format!(" · {reference}.{} on {}", pad.number, net.name)
                                }
                                (None, true) => format!(" · mechanical pad of {reference}"),
                                (None, false) => format!(" · {reference}.{}, no net", pad.number),
                            }
                        })
                        .unwrap_or_default()
                }
                Some(BoardSelection::Silk(id, index)) => {
                    format!(" · silkscreen line {index} of {}", self.reference_of(id))
                }
                Some(BoardSelection::Zone(id)) => self
                    .document
                    .board
                    .zones
                    .iter()
                    .find(|z| z.id == id)
                    .map(|z| format!(" · {}", zones::describe(z, self.document.board.layer_count)))
                    .unwrap_or_default(),
                _ => String::new(),
            };
            // A draft too close to another net says so on the line the pointer
            // is read from, as the preview turns the warning colour.
            let clearance = leg.as_ref().and_then(|leg| leg.clash.as_ref()).map_or_else(String::new, |clash| {
                if clash.crosses() {
                    format!(" · short: {}", clash.words())
                } else {
                    format!(" · clearance: {}, {}", clash.words(), clash.needed_words())
                }
            });
            // A part being dragged onto another net's copper or another part says so
            // on the same line, as a draft does, before it is let go.
            let placing = match (&self.board_view.placing, self.board_view.drag.as_ref().map(|d| d.3)) {
                (Some((_, landfall)), Some(Subject::Part(id))) => landfall
                    .words(self.reference_of(id))
                    .map_or_else(String::new, |words| format!(" · {words}")),
                _ => String::new(),
            };
            self.status = format!(
                "PCB · {:.2}, {:.2} mm · {} · {} unrouted · {:.0}%{under}{clearance}{placing}",
                mm(q.x),
                mm(q.y),
                layer_name(
                    self.board_view.active_layer,
                    self.document.board.layer_count
                ),
                airwires.len(),
                self.board_view.zoom / 0.012 * 100.
            );
        }
        self.poll_autoroute(&ctx);
    }
    fn board_input(&mut self, ui: &mut egui::Ui, response: &egui::Response, pointer: Option<Pos2>) {
        // A line in the Inspector is news about the last thing done, and it is spent
        // by the next press on the board, as the sheet's notice is: whatever that
        // press goes on to do, the user has moved on. First in the frame, so a line
        // this frame says (a key's refusal, the release that ends a drag) is kept.
        if response.hovered() && ui.input(|i| i.pointer.primary_pressed()) {
            self.board_view.message.clear();
        }
        // The wheel zooms about the pointer and needs it on the board; the keys do
        // not (`keys_reach_editor`).
        if response.hovered() && !ui.ctx().egui_wants_keyboard_input() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.
                && let Some(p) = pointer
            {
                self.board_view.zoom_about((scroll * 0.002).exp(), p);
            }
        }
        if crate::keys_reach_editor(ui.ctx()) {
            self.run_shortcuts(ui.ctx());
        }
        if response.dragged_by(PointerButton::Middle)
            || response.dragged_by(PointerButton::Secondary)
        {
            self.board_view.pan += ui.input(|i| i.pointer.delta());
        }
        if response.secondary_clicked() {
            self.board_view.draft = None;
            self.board_view.selected = pointer.and_then(|p| self.hit_board(p));
        }
        response.context_menu(|ui| {
            menu_ui(ui);
            match self.board_view.selected {
                Some(
                    BoardSelection::Part(_) | BoardSelection::Pad(..) | BoardSelection::Silk(..),
                ) => {
                    // A pad's menu is its part's, with the pad named at the top so
                    // a right-click that landed on a pad says what it caught, and
                    // one entry to step up to the whole part.
                    if let Some(BoardSelection::Pad(id, index)) = self.board_view.selected {
                        let title = self
                            .pad_at(id, index)
                            .map(|(reference, pad)| pad_title(reference, pad));
                        if let Some(title) = title {
                            ui.label(egui::RichText::new(title).small());
                        }
                        let reference = self.reference_of(id).to_owned();
                        let select = ui.button(format!("Select {reference}"));
                        self.board_view.menu.push(("menuitem:select_part".into(), select.rect));
                        if select.clicked() {
                            self.board_view.selected = Some(BoardSelection::Part(id));
                            ui.close();
                        }
                        let route = ui.button("Route from this pad");
                        self.board_view.menu.push(("menuitem:route_from_here".into(), route.rect));
                        if route.clicked() {
                            self.set_board_tool(BoardTool::Route);
                            ui.close();
                        }
                    }
                    // A silk line's menu is its part's too, named the same way.
                    if let Some(BoardSelection::Silk(id, index)) = self.board_view.selected {
                        let reference = self.reference_of(id).to_owned();
                        ui.label(
                            egui::RichText::new(format!("{reference} · silkscreen line {index}"))
                                .small(),
                        );
                        let select = ui.button(format!("Select {reference}"));
                        self.board_view.menu.push(("menuitem:select_part".into(), select.rect));
                        if select.clicked() {
                            self.board_view.selected = Some(BoardSelection::Part(id));
                            ui.close();
                        }
                    }
                    let rotate = ui.button("Rotate 90°");
                    self.board_view.menu.push(("menuitem:rotate".into(), rotate.rect));
                    if rotate.clicked() {
                        self.rotate_part();
                        ui.close();
                    }
                    let flip = ui.button("Flip side");
                    self.board_view.menu.push(("menuitem:flip_side".into(), flip.rect));
                    if flip.clicked() {
                        self.flip_part();
                        ui.close();
                    }
                }
                Some(selected) => {
                    if matches!(selected, BoardSelection::Zone(_)) {
                        let fill = ui.button("Fill zones");
                        self.board_view.menu.push(("menuitem:fill_zones".into(), fill.rect));
                        if fill.clicked() {
                            self.fill_zones();
                            ui.close();
                        }
                    }
                    if matches!(selected, BoardSelection::Via(_)) {
                        let route = ui.button("Route from this via");
                        self.board_view.menu.push(("menuitem:route_from_here".into(), route.rect));
                        if route.clicked() {
                            self.set_board_tool(BoardTool::Route);
                            ui.close();
                        }
                    }
                    let delete = ui.button("Delete");
                    self.board_view.menu.push(("menuitem:delete".into(), delete.rect));
                    if delete.clicked() {
                        self.delete_board_selection();
                        ui.close();
                    }
                }
                None => {
                    ui.close();
                }
            }
        });
        match self.board_view.tool {
            BoardTool::Select => {
                // Say what a press would take hold of before it is pressed.
                if self.board_view.drag.is_some() {
                    ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
                } else if let Some(p) = pointer
                    && self.grabbable(p)
                {
                    ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
                }
                if response.drag_started_by(PointerButton::Primary)
                    && let Some(p) = ui.input(|i| i.pointer.press_origin())
                {
                    let previous = self.board_view.selected;
                    self.board_view.selected = self.hit_board(p);
                    // An edge's middle puts a corner in only on a zone already
                    // selected, whose middles are drawn; on any other it drags it.
                    let zone_grip = match self.board_view.selected {
                        Some(BoardSelection::Zone(id)) => self.zone_grip(id, p).map(|g| match g {
                            ZoneGrip::Edge(_) if previous != Some(BoardSelection::Zone(id)) => ZoneGrip::Body,
                            g => g,
                        }),
                        _ => None,
                    };
                    // Parts, vias and tracks are all dragged. Dragging a PAD drags
                    // the part it belongs to — a pad cannot move on its own, and
                    // grabbing one is how a part with pads over most of its body is
                    // grabbed — while the SELECTION stays the pad the user pressed on.
                    // A silkscreen line is the same: it drags its part.
                    // A track is grabbed by a corner or by a segment; see `grip`.
                    let q = self.board_view.world(p);
                    let board = &self.document.board;
                    let grabbed = match self.board_view.selected {
                        Some(
                            BoardSelection::Part(id)
                            | BoardSelection::Pad(id, _)
                            | BoardSelection::Silk(id, _),
                        ) => board.placement(id).map(|p| (Subject::Part(id), p.at)),
                        Some(BoardSelection::Via(id)) => board
                            .vias
                            .iter()
                            .find(|v| v.id == id)
                            .map(|v| (Subject::Via(id), v.at)),
                        Some(BoardSelection::Track(id)) => board
                            .tracks
                            .iter()
                            .find(|t| t.id == id)
                            .and_then(|t| grip(board, t, q, self.board_view.tolerance()))
                            .map(|g| (Subject::Track(id, g), q)),
                        Some(BoardSelection::Zone(id)) => zone_grip.map(|g| (Subject::Zone(id, g), q)),
                        None => None,
                    };
                    if let Some((what, at)) = grabbed {
                        self.board_view.drag = Some((self.document.clone(), q, at, what));
                        self.board_view.landing = None;
                        self.board_view.placing = matches!(what, Subject::Part(_)).then(|| {
                            let conn = self.document.board.connectivity(&self.document.netlist());
                            (conn, Landfall::default())
                        });
                    }
                }
                if let (Some((before, start, at, what)), Some(p)) = (&self.board_view.drag, pointer)
                {
                    let q = self.board_view.world(p);
                    // Snap the offset, not the position, so what moves keeps its existing
                    // alignment: a part stays where it sat between the grid's lines, and a
                    // via or a track keeps whatever pitch the autorouter left it on.
                    let raw = Point::new(q.x - start.x, q.y - start.y);
                    let mut delta = snap(raw, PLACE_GRID);
                    // A free END over a pad of its own net goes to the pad's centre
                    // rather than to the nearest grid step, which lands on a pad's
                    // centre only when the geometry happens to put it there.
                    let landing = match *what {
                        Subject::Track(id, Grip::Vertex(i)) => before
                            .board
                            .tracks
                            .iter()
                            .find(|t| t.id == id)
                            .and_then(|t| t.points.get(i))
                            .and_then(|end| land_end(before, id, i, end.offset(raw))),
                        _ => None,
                    };
                    if let Some(landing) = landing.filter(|l| l.joins) {
                        delta = landing.delta;
                    }
                    self.board_view.landing = landing;
                    let (target, what) = (at.offset(delta), *what);
                    self.document = before.clone();
                    let board = &mut self.document.board;
                    match what {
                        Subject::Part(id) => {
                            if let Some(p) = board.placements.iter_mut().find(|p| p.component == id)
                            {
                                p.at = target;
                            }
                        }
                        Subject::Via(id) => move_via(board, id, delta),
                        Subject::Track(id, Grip::Vertex(i)) => move_vertex(board, id, i, delta),
                        Subject::Track(id, Grip::Segment(k)) => slide_segment(board, id, k, delta),
                        // The fill is taken away while the outline moves, and made
                        // again when it is let go: a fill that trailed the outline
                        // would show copper the zone no longer asks for.
                        Subject::Zone(id, grip) if delta != Point::default() => {
                            if let Some(zone) = board.zones.iter_mut().find(|z| z.id == id) {
                                zone.outline = dragged_outline(&zone.outline, grip, delta);
                                zone.fill.clear();
                            }
                        }
                        Subject::Zone(..) => {}
                    }
                    if let Subject::Part(id) = what
                        && let Some((before, _)) = &self.board_view.placing
                    {
                        let landfall = self.landfall(id, before);
                        if let Some((_, now)) = &mut self.board_view.placing {
                            *now = landfall;
                        }
                    }
                }
                if response.drag_stopped_by(PointerButton::Primary)
                    && let Some((before, ..)) = self.board_view.drag.take()
                {
                    // An end left on another net's pad is not snapped, and says why.
                    if let Some(landing) = self.board_view.landing.take()
                        && !landing.joins
                        && let Some((reference, pad)) =
                            self.pad_at(landing.component, landing.index)
                    {
                        let on = self
                            .pad_net(landing.component, &pad.number)
                            .map_or_else(|| "no net".to_owned(), |net| format!("net {net}"));
                        self.board_view.message = format!(
                            "{} is on {on}, not this track's: the end was left on its grid step instead of snapped to the pad's centre. Check the board to see the short.",
                            pad_title(reference, pad)
                        );
                    }
                    // A part put down on another net's copper, or on another part, is
                    // put down, as a route that clears no corner is drawn; and the
                    // line the status bar showed while it moved stays in the
                    // Inspector, where it outlasts the drag.
                    if let Some((_, landfall)) = self.board_view.placing.take()
                        && let Some(BoardSelection::Part(id) | BoardSelection::Pad(id, _) | BoardSelection::Silk(id, _)) =
                            self.board_view.selected
                        && before.board != self.document.board
                    {
                        let reference = self.reference_of(id).to_owned();
                        if let Some(words) = landfall.words(&reference) {
                            let mut words = words;
                            words[..1].make_ascii_uppercase();
                            self.board_view.message = format!(
                                "{words}. {reference} was put down there; move it clear, or Undo puts it back."
                            );
                        }
                    }
                    // A zone moved is refilled, all zones with it — the one it now
                    // overlaps may be first in line — inside the same undo step.
                    let zone_moved = before.board.zones.len() == self.document.board.zones.len()
                        && before
                            .board
                            .zones
                            .iter()
                            .zip(&self.document.board.zones)
                            .any(|(a, b)| a.outline != b.outline);
                    if zone_moved {
                        let reports = self.document.fill_zones();
                        self.note_filled();
                        let words = self.zone_reports_words(&reports);
                        if let Some(BoardSelection::Zone(id)) = self.board_view.selected
                            && let Some(k) = self.document.board.zones.iter().position(|z| z.id == id)
                        {
                            self.board_view.message = words.get(k).cloned().unwrap_or_default();
                        }
                    }
                    self.commit(before, None);
                }
                if response.clicked_by(PointerButton::Primary)
                    && let Some(p) = pointer
                {
                    self.board_view.selected = self.hit_board(p);
                }
            }
            BoardTool::Zone => {
                if response.double_clicked_by(PointerButton::Primary) {
                    self.finish_zone();
                } else if response.clicked_by(PointerButton::Primary)
                    && let Some(p) = pointer
                {
                    self.zone_click(p);
                }
            }
            BoardTool::Route => {
                if response.double_clicked_by(PointerButton::Primary) {
                    self.finish_draft();
                } else if response.clicked_by(PointerButton::Primary)
                    && let Some(p) = pointer
                {
                    self.route_click(p);
                }
            }
        }
    }

    fn paint_board(&self, painter: &egui::Painter, airwires: &[Airwire], pointer: Option<Pos2>, leg: Option<&Leg>) {
        let view = &self.board_view;
        let board = &self.document.board;
        let count = board.layer_count;
        painter.rect_filled(view.canvas, 0., BACKGROUND);
        let outline: Vec<Pos2> = board.outline.iter().map(|p| view.screen(*p)).collect();
        if board.outline_is_rectangle() {
            painter.rect_filled(Rect::from_points(&outline), 0., SUBSTRATE);
        }
        painter.add(egui::Shape::closed_line(outline, Stroke::new(1.5, EDGE)));
        let paint_shape = |shape: &Shape, color: Color32| match *shape {
            Shape::Rect { min, max } => {
                painter.rect_filled(
                    Rect::from_two_pos(view.screen(min), view.screen(max)),
                    0.,
                    color,
                );
            }
            Shape::Capsule { a, b, radius } => {
                let r = view.px(radius).max(0.5);
                let (a, b) = (view.screen(a), view.screen(b));
                if a != b {
                    painter.line_segment([a, b], Stroke::new(2. * r, color));
                    painter.circle_filled(b, r, color);
                }
                painter.circle_filled(a, r, color);
            }
        };
        let paint_track = |track: &Track, color: Color32| {
            for shape in track.segments() {
                paint_shape(&shape, color);
            }
        };
        let routed: Vec<(&[Track], &[Via])> = view.job.iter().flat_map(|j| j.routed()).collect();
        let active = view.active_layer.min(count - 1);
        let layers: Vec<u8> = (0..count)
            .rev()
            .filter(|l| *l != active)
            .chain(std::iter::once(active))
            .filter(|l| view.visible(*l))
            .collect();
        let stale = self.zones_stale();
        for &layer in &layers {
            let base = layer_color(layer, count);
            let color = if layer == active {
                base.gamma_multiply(0.9)
            } else {
                base.gamma_multiply(0.45)
            };
            // A zone under everything else on its layer: its fill as the layer's
            // colour, see-through, and its outline dashed round it.
            for zone in board.zones.iter().filter(|z| z.layer == layer) {
                // Fainter than the layer's tracks and pads, which are drawn over
                // it in the layer's full colour, so copper on the zone's net still
                // reads as tracks and pads and not as more fill.
                let fill = if layer == active { base.gamma_multiply(0.3) } else { base.gamma_multiply(0.14) };
                let mut mesh = egui::epaint::Mesh::default();
                for piece in &zone.fill {
                    let rings: Vec<&[Point]> = std::iter::once(piece.outer.as_slice())
                        .chain(piece.holes.iter().map(Vec::as_slice))
                        .collect();
                    for tri in zones::fill_triangles(&rings) {
                        let base_index = mesh.vertices.len() as u32;
                        for (x, y) in tri {
                            let at = view.canvas.center() + view.pan + Vec2::new(x as f32, y as f32) * view.zoom;
                            mesh.colored_vertex(at, fill);
                        }
                        mesh.add_triangle(base_index, base_index + 1, base_index + 2);
                    }
                }
                painter.add(egui::Shape::mesh(mesh));
                let mut ring: Vec<Pos2> = zone.outline.iter().map(|p| view.screen(*p)).collect();
                ring.extend(ring.first().copied());
                let edge = if stale { WARNING.gamma_multiply(0.8) } else { base.gamma_multiply(0.8) };
                painter.extend(egui::Shape::dashed_line(&ring, Stroke::new(1.2, edge), 6., 4.));
            }
            for placement in &board.placements {
                for pad in placement
                    .footprint
                    .pads
                    .iter()
                    .filter(|p| p.drill.is_none())
                {
                    if placement.pad_layers(pad, count).0 == layer {
                        paint_shape(&placement.pad_shape(pad), color);
                    }
                }
            }
            for track in board.tracks.iter().filter(|t| t.layer == layer) {
                let selected = view.selected == Some(BoardSelection::Track(track.id));
                paint_track(
                    track,
                    if selected {
                        base.lerp_to_gamma(Color32::WHITE, 0.5)
                    } else {
                        color
                    },
                );
            }
            for (tracks, _) in &routed {
                for track in tracks.iter().filter(|t| t.layer == layer) {
                    paint_track(track, color);
                }
            }
            if let Some(draft) = &view.draft {
                for track in draft.tracks.iter().filter(|t| t.layer == layer) {
                    paint_track(track, base);
                }
            }
        }
        let paint_via = |via: &Via, selected: bool| {
            let at = view.screen(via.at);
            painter.circle_filled(
                at,
                view.px(via.diameter / 2).max(1.5),
                if selected { ACCENT } else { VIA },
            );
            painter.circle_filled(at, view.px(via.drill / 2).max(0.8), BACKGROUND);
        };
        for placement in &board.placements {
            for pad in placement.footprint.pads.iter() {
                if let Some(drill) = pad.drill {
                    paint_shape(&placement.pad_shape(pad), THROUGH_HOLE);
                    painter.circle_filled(
                        view.screen(placement.transform(pad.at)),
                        view.px(drill / 2).max(0.8),
                        BACKGROUND,
                    );
                }
            }
        }
        for via in &board.vias {
            paint_via(via, view.selected == Some(BoardSelection::Via(via.id)));
        }
        for (_, vias) in &routed {
            for via in *vias {
                paint_via(via, false);
            }
        }
        if let Some(draft) = &view.draft {
            for via in &draft.vias {
                paint_via(via, true);
            }
        }
        // The selected track's handles: a square on every corner a drag can take,
        // and a ring on each end that is held where it is.
        if let Some(BoardSelection::Track(id)) = view.selected
            && let Some(track) = board.tracks.iter().find(|t| t.id == id)
        {
            // Sized from the track's own width, or a fat track hides them.
            let size = (view.px(track.width) + 5.).max(9.);
            let last = track.points.len().saturating_sub(1);
            for (i, &p) in track.points.iter().enumerate() {
                let at = view.screen(p);
                if (i == 0 || i == last) && held(board, track, p) {
                    painter.circle_stroke(at, size / 2., Stroke::new(1.5, ACCENT));
                } else {
                    let handle = Rect::from_center_size(at, Vec2::splat(size));
                    painter.rect_filled(handle, 1., BACKGROUND);
                    painter.rect_stroke(
                        handle,
                        1.,
                        Stroke::new(1.5, ACCENT),
                        egui::StrokeKind::Inside,
                    );
                }
            }
        }
        // The selected zone: its outline in the accent, a square on every corner
        // and a dot on every edge's middle, which a drag turns into a corner.
        if let Some(BoardSelection::Zone(id)) = view.selected
            && let Some(zone) = board.zones.iter().find(|z| z.id == id)
        {
            let mut ring: Vec<Pos2> = zone.outline.iter().map(|p| view.screen(*p)).collect();
            ring.extend(ring.first().copied());
            painter.add(egui::Shape::line(ring, Stroke::new(1.5, ACCENT)));
            for &c in &zone.outline {
                let handle = Rect::from_center_size(view.screen(c), Vec2::splat(9.));
                painter.rect_filled(handle, 1., BACKGROUND);
                painter.rect_stroke(handle, 1., Stroke::new(1.5, ACCENT), egui::StrokeKind::Inside);
            }
            for m in edge_middles(&zone.outline) {
                painter.circle_filled(view.screen(m), 3.5, ACCENT.gamma_multiply(0.8));
            }
            if stale {
                painter.text(
                    view.screen(zone.outline[0]) + Vec2::new(8., -8.),
                    Align2::LEFT_BOTTOM,
                    stale_words(board),
                    FontId::proportional(12.),
                    WARNING,
                );
            }
        } else if stale && let Some(zone) = board.zones.first() {
            // With no zone selected, the same words at the first zone's first
            // corner: a fill saved stale showed only a dashed outline in the
            // warning colour, which on a pour drawn to the board's edge lies
            // under the board outline and cannot be seen.
            painter.text(
                view.screen(zone.outline[0]) + Vec2::new(8., -8.),
                Align2::LEFT_BOTTOM,
                stale_words(board),
                FontId::proportional(12.),
                WARNING,
            );
        }
        // The zone being drawn: its corners so far, the edge to the pointer, and
        // the closing edge back to the first corner dashed.
        if view.tool == BoardTool::Zone && !view.zone_draft.is_empty() {
            let colour = layer_color(active, count);
            let mut points: Vec<Pos2> = view.zone_draft.iter().map(|p| view.screen(*p)).collect();
            if let Some(p) = pointer {
                points.push(view.screen(snap(view.world(p), PLACE_GRID)));
            }
            painter.add(egui::Shape::line(points.clone(), Stroke::new(1.5, colour)));
            if points.len() >= 3 {
                painter.extend(egui::Shape::dashed_line(
                    &[*points.last().unwrap(), points[0]],
                    Stroke::new(1., colour.gamma_multiply(0.7)),
                    5.,
                    4.,
                ));
            }
            for (i, &c) in view.zone_draft.iter().enumerate() {
                let at = view.screen(c);
                painter.circle_filled(at, if i == 0 { 5. } else { 3.5 }, colour);
            }
        }
        if let Some(p) = pointer
            && view.tool == BoardTool::Zone
        {
            let at = view.screen(snap(view.world(p), PLACE_GRID));
            painter.circle_stroke(at, 5., Stroke::new(1.5, ACCENT));
            painter.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        }
        // The pad a dragged free end is coming down on: the accent where it
        // snaps to the centre, the warning colour where it is another net's.
        if view.drag.is_some()
            && let Some(landing) = view.landing
            && let Some(placement) = board.placement(landing.component)
            && let Some(pad) = placement.footprint.pads.get(landing.index)
        {
            let (min, max) = placement.pad_shape(pad).bounds();
            painter.rect_stroke(
                Rect::from_two_pos(view.screen(min), view.screen(max)).expand(3.),
                2.,
                Stroke::new(2., if landing.joins { ACCENT } else { WARNING }),
                egui::StrokeKind::Outside,
            );
        }
        let references: BTreeMap<Uuid, &str> = self
            .document
            .components
            .iter()
            .map(|c| (c.id, c.reference.as_str()))
            .collect();
        for placement in &board.placements {
            let silk = if placement.bottom {
                Color32::from_rgb(142, 154, 186)
            } else {
                SILK
            };
            let (min, max) = placement.courtyard();
            let bounds = Rect::from_two_pos(view.screen(min), view.screen(max));
            for (index, line) in placement.footprint.silk.iter().enumerate() {
                // A selected line wears the accent, and its part the dimmer
                // outline a selected pad gives it: legend is always legend OF a part.
                let chosen =
                    view.selected == Some(BoardSelection::Silk(placement.component, index));
                if chosen {
                    painter.rect_stroke(
                        bounds,
                        2.,
                        Stroke::new(1., ACCENT.gamma_multiply(0.4)),
                        egui::StrokeKind::Outside,
                    );
                }
                painter.add(egui::Shape::line(
                    line.iter()
                        .map(|p| view.screen(placement.transform(*p)))
                        .collect(),
                    if chosen {
                        Stroke::new(2.5, ACCENT)
                    } else {
                        Stroke::new(1., silk.gamma_multiply(0.7))
                    },
                ));
            }
            if bounds.width() > 12. {
                painter.text(
                    Pos2::new(bounds.center().x, bounds.top() - 2.),
                    Align2::CENTER_BOTTOM,
                    references.get(&placement.component).copied().unwrap_or("?"),
                    FontId::proportional((view.zoom * 1000.).clamp(9., 16.)),
                    silk,
                );
            }
            // A pad carries its number as soon as it is big enough on screen to hold
            // one, which is how the footprint editor shows them.
            for pad in placement
                .footprint
                .pads
                .iter()
                .filter(|p| !p.number.is_empty())
            {
                let Some(height) = pad_number_height(pad.size, view.zoom) else {
                    continue;
                };
                painter.text(
                    view.screen(placement.transform(pad.at)),
                    Align2::CENTER_CENTER,
                    &pad.number,
                    FontId::monospace(height),
                    Color32::WHITE,
                );
            }
            if view.selected == Some(BoardSelection::Part(placement.component)) {
                painter.rect_stroke(
                    bounds,
                    2.,
                    Stroke::new(1.5, ACCENT),
                    egui::StrokeKind::Outside,
                );
            }
            // A selected PAD wears the accent itself, and its part keeps a
            // dimmer line of the same colour: a pad is always a pad OF
            // something, and at board zoom the part is what a user recognises.
            if let Some(BoardSelection::Pad(component, index)) = view.selected
                && component == placement.component
                && let Some(pad) = placement.footprint.pads.get(index)
            {
                painter.rect_stroke(
                    bounds,
                    2.,
                    Stroke::new(1., ACCENT.gamma_multiply(0.4)),
                    egui::StrokeKind::Outside,
                );
                let (min, max) = placement.pad_shape(pad).bounds();
                painter.rect_stroke(
                    Rect::from_two_pos(view.screen(min), view.screen(max)).expand(2.),
                    1.,
                    Stroke::new(2., ACCENT),
                    egui::StrokeKind::Outside,
                );
            }
        }
        if view.show_ratsnest {
            for wire in airwires {
                painter.line_segment(
                    [view.screen(wire.a), view.screen(wire.b)],
                    Stroke::new(1., Color32::from_rgba_unmultiplied(236, 236, 236, 150)),
                );
            }
        }
        if let Some(violations) = &view.violations {
            for (i, v) in violations
                .iter()
                .enumerate()
                .filter(|(_, v)| v.kind != ViolationKind::Unrouted)
            {
                let at = view.screen(v.at);
                // The finding the list was last clicked on wears a second, wider
                // ring, so it is the one the eye finds among its neighbours.
                if view.focused_violation == Some(i) {
                    painter.circle_stroke(at, 13., Stroke::new(2.5, WARNING));
                }
                painter.circle_stroke(at, 7., Stroke::new(2., WARNING));
                painter.line_segment(
                    [at - Vec2::splat(4.), at + Vec2::splat(4.)],
                    Stroke::new(2., WARNING),
                );
                painter.line_segment(
                    [at + Vec2::new(-4., 4.), at + Vec2::new(4., -4.)],
                    Stroke::new(2., WARNING),
                );
            }
        }
        if let (Some(draft), Some(leg)) = (&view.draft, leg) {
            let mut points = draft.points.clone();
            points.extend(leg.points.iter().skip(1).copied());
            let preview = Track {
                id: Uuid::nil(),
                layer: draft.layer,
                width: draft.width,
                fixed_width: false,
                points,
            };
            // Too close to another net: the preview wears the warning colour,
            // and the copper it is too close to is ringed, before any click.
            let colour = if leg.clash.is_some() { WARNING } else { layer_color(draft.layer, count) };
            paint_track(&preview, colour.gamma_multiply(0.85));
            if let Some(clash) = &leg.clash {
                let at = view.screen(clash.at);
                painter.circle_stroke(at, 11., Stroke::new(2., WARNING));
                painter.text(
                    at + Vec2::new(14., -14.),
                    Align2::LEFT_BOTTOM,
                    if clash.crosses() { format!("short: {}", clash.words()) } else { format!("{} — clearance", clash.words()) },
                    FontId::proportional(13.),
                    WARNING,
                );
            }
        }
        if let Some(p) = pointer
            && view.tool == BoardTool::Route
        {
            let (target, anchor) = self.route_target(p);
            let at = view.screen(target);
            painter.circle_stroke(
                at,
                if anchor.is_some() { 8. } else { 5. },
                Stroke::new(1.5, ACCENT),
            );
            painter.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        }
        // A part being dragged onto another net's copper or another part: its
        // courtyard and what it would come down on wear the warning colour, and
        // the copper is ringed as a draft's clash is. Last, over the selection's
        // own outline, which would otherwise hide it.
        if let (Some((_, landfall)), Some((.., Subject::Part(id)))) = (&view.placing, &view.drag)
            && !landfall.is_clear()
            && let Some(placement) = board.placement(*id)
        {
            let (min, max) = placement.courtyard();
            painter.rect_stroke(
                Rect::from_two_pos(view.screen(min), view.screen(max)),
                2.,
                Stroke::new(2., WARNING),
                egui::StrokeKind::Outside,
            );
            for (_, clash) in &landfall.copper {
                painter.circle_stroke(view.screen(clash.at), 11., Stroke::new(2., WARNING));
            }
            for (other, _) in &landfall.courtyards {
                if let Some(other) = board.placements.iter().find(|p| self.reference_of(p.component) == other) {
                    let (min, max) = other.courtyard();
                    painter.rect_stroke(
                        Rect::from_two_pos(view.screen(min), view.screen(max)),
                        2.,
                        Stroke::new(1.5, WARNING.gamma_multiply(0.8)),
                        egui::StrokeKind::Outside,
                    );
                }
            }
        }
        if board.placements.is_empty() {
            painter.text(
                view.canvas.center(),
                Align2::CENTER_CENTER,
                "No parts on the board yet\nAdd native assembly parts with pads, then choose Update from parts",
                FontId::proportional(18.),
                Color32::from_rgb(119, 137, 158),
            );
        }
    }
}

