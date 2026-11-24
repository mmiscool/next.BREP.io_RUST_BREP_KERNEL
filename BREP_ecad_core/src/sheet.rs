//! One sheet over a board's schematic and the harness wiring round it.
//!
//! A host keeps an assembly's eCAD in two documents: a schematic (the board this
//! assembly is: its parts, its nets and its layout) and a wiring diagram (the parts
//! off that board and the harness wires between them). The user edits them as ONE
//! sheet, a [`DocumentKind::Sheet`]: the board is drawn as a zone round the parts on
//! it, and the rest of the sheet holds the parts off it.
//!
//! - [`Document::merge_sheet`] reads the two into a sheet, and [`Document::split_sheet`]
//!   writes a sheet back as the two. A split of a sheet nothing has edited gives back
//!   documents EQUAL to the ones merged, so a host that writes only the documents that
//!   changed writes nothing for an open.
//! - A part on BOTH documents (a connector with nets on the board and a harness wire
//!   off it) is ONE component on the sheet, matched by its assembly occurrence. The
//!   wiring diagram keeps its own copy of it, its TWIN, which is where its harness
//!   wires land when the diagram is stored.
//! - The two documents were drawn on separate canvases, so the diagram's content is
//!   moved onto the sheet by an offset: the diagram's [`Document::board_zone`] once it
//!   has been written from a sheet, else one the host keeps for the session, else one
//!   that puts the diagram's parts to the left of the board's.
//!
//! - A board of its own that the assembly places (a sub-board) is a ZONE of the sheet
//!   ([`SheetZone`]): its schematic, drawn read-only where the diagram says
//!   ([`Document::zones`]), else beside everything else. Its parts and nets belong to
//!   its own document and are never written to this one's; a harness wire may end on
//!   a pin inside it only where the board declares a point for that pin.
//!
//! The sheet's own rules ([`Document::validate`]): a net joins only parts on the board
//! (owner 2026-11-22: "nets should only work inside of a board zone"); a harness wire
//! joins two pins, and a pin takes one harness wire, though it may carry a net as well.
use crate::{Component, Document, DocumentKind, Point, Terminal, Uuid, Wire};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// A board of its own on a sheet: one placement of a sub-board.
///
/// A wiring diagram stores `occurrence` and `at` alone (where the zone is drawn, in
/// the diagram's coordinates). On a sheet `at` is in the sheet's coordinates and the
/// rest says what the zone holds.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SheetZone {
    /// The sub-board's assembly occurrence (`ACOMP9`).
    pub occurrence: String,
    /// Where the board's schematic origin is drawn.
    pub at: Point,
    /// On a sheet: the board's label (`A1`).
    #[serde(skip)]
    pub reference: String,
    /// On a sheet: the parts drawn from the board's schematic.
    #[serde(skip)]
    pub members: Vec<Uuid>,
    /// On a sheet: the board's nets, drawn from its schematic.
    #[serde(skip)]
    pub nets: Vec<Uuid>,
    /// On a sheet: the pins of `members` the board declares a point for, the only
    /// pins inside it a harness wire may end on.
    #[serde(skip)]
    pub declared: Vec<Terminal>,
    /// On a sheet: the board's net labels, drawn from its schematic.
    #[serde(skip)]
    pub labels: Vec<Uuid>,
    /// On a sheet: the board's junctions, where the sheet draws them.
    #[serde(skip)]
    pub junctions: Vec<Point>,
}

/// A sub-board as a host hands it to [`Document::merge_sheet_with`].
#[derive(Clone, Debug, PartialEq)]
pub struct SubBoard {
    /// Its occurrence in this assembly (`ACOMP9`).
    pub occurrence: String,
    /// Its label (`A1`).
    pub reference: String,
    /// Its own schematic, its `pcb` block.
    pub schematic: Document,
    /// The pins of `schematic`'s parts the board declares a point for.
    pub declared: Vec<Terminal>,
}

/// A sub-board's part or net id on a sheet: its own id, made distinct per placement
/// so two placements of one board do not share ids.
fn zone_id(occurrence: &str, id: Uuid) -> Uuid {
    let mut h: u128 = 0x6c62_272e_07bb_0142_62b8_2175_6295_c58d;
    for b in occurrence.bytes() {
        h ^= b as u128;
        h = h.wrapping_mul(0x0000_0000_0100_0000_0000_0000_0000_013b);
    }
    Uuid::from_u128(id.as_u128() ^ h)
}

/// Format version of a sheet. It is never stored by a host, which stores the two
/// documents it splits into; newer than both so an older build refuses one.
pub const SHEET_DOCUMENT_VERSION: u32 = 6;

/// The gap left between the diagram's parts and the board's when a merge places
/// them side by side: 25.4 mm.
const SIDE_BY_SIDE_GAP: i32 = 25_400;

/// What [`Document::merge_sheet`] read, kept by the host for the split.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SheetSource {
    /// The schematic as merged; `None` when the assembly has no board of its own.
    pub pcb: Option<Document>,
    /// The wiring diagram as merged; `None` when it has none.
    pub diagram: Option<Document>,
    /// Where the board's schematic origin sits in the diagram's coordinates: a diagram
    /// point `p` is drawn at `p - offset` on the sheet.
    pub offset: Point,
    /// A part on both documents: the sheet component's id (the schematic's), and the
    /// diagram's own copy of it as merged.
    pub twins: BTreeMap<Uuid, Component>,
    /// A harness wire whose end the merge moved onto a part's place on the board
    /// (its twin sat elsewhere): the diagram's wire, and the sheet's as merged. An
    /// unedited one splits back to the diagram's.
    pub moved_wires: BTreeMap<Uuid, (Wire, Wire)>,
    /// The sub-board zones as merged, in the sheet's coordinates.
    pub zones: Vec<SheetZone>,
}

/// Move a wire's ends onto the pins it names in `doc`, where they are elsewhere.
fn reattach(doc: &Document, w: &mut Wire) {
    let a = w.start.as_ref().and_then(|t| doc.terminal_position(t)).unwrap_or(w.a);
    let b = w.end.as_ref().and_then(|t| doc.terminal_position(t)).unwrap_or(w.b);
    if (a, b) != (w.a, w.b) {
        w.follow(a, b);
    }
}

/// A sheet written back as its two documents.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SplitSheet {
    pub pcb: Option<Document>,
    pub diagram: Option<Document>,
    /// The sheet's id of every part the diagram holds → the diagram's id for it.
    pub diagram_ids: BTreeMap<Uuid, Uuid>,
}

impl SheetSource {
    /// What a merge of `split` would have read, for the sheet `sheet` it was split
    /// from: the source the NEXT split of the same sheet is taken against, once the
    /// host has stored `split`. The sheet keeps its own ids, so its editor keeps its
    /// selection; re-merging would rename a part that moved off the board.
    pub fn after(sheet: &Document, split: &SplitSheet, offset: Point) -> SheetSource {
        let mut twins = BTreeMap::new();
        let mut moved_wires = BTreeMap::new();
        if let Some(d) = &split.diagram {
            for (sheet_id, diagram_id) in &split.diagram_ids {
                let on_board = sheet.components.iter().any(|c| c.id == *sheet_id && (!c.off_board || sheet.zone_member(c.id).is_some()));
                if (on_board || sheet_id != diagram_id)
                    && let Some(copy) = d.components.iter().find(|c| c.id == *diagram_id)
                {
                    twins.insert(*sheet_id, copy.clone());
                }
            }
            let back: BTreeMap<Uuid, Uuid> = split.diagram_ids.iter().map(|(s, d)| (*d, *s)).collect();
            for dw in &d.wires {
                let Some(w) = sheet.wires.iter().find(|w| w.id == dw.id) else { continue };
                let mut merged = dw.clone();
                shift_wire(&mut merged, neg(offset));
                for t in [&mut merged.start, &mut merged.end].into_iter().flatten() {
                    if let Some(id) = back.get(&t.component) {
                        t.component = *id;
                    }
                }
                merged.harness = true;
                if merged != *w {
                    moved_wires.insert(w.id, (dw.clone(), w.clone()));
                }
            }
        }
        SheetSource {
            pcb: split.pcb.clone(),
            diagram: split.diagram.clone(),
            offset,
            twins,
            moved_wires,
            zones: sheet.zones.clone(),
        }
    }
}

fn shift_component(c: &mut Component, by: Point) {
    c.at = c.at.offset(by);
    for g in &mut c.gates {
        g.at = g.at.offset(by);
    }
}

fn shift_wire(w: &mut Wire, by: Point) {
    w.a = w.a.offset(by);
    w.b = w.b.offset(by);
    for p in &mut w.bends {
        *p = p.offset(by);
    }
}

fn neg(p: Point) -> Point {
    Point::new(-p.x, -p.y)
}

fn extent_of(components: &[Component]) -> Option<(Point, Point)> {
    let boxes: Vec<(Point, Point)> =
        components.iter().flat_map(Component::sheet_boxes).map(|(_, a, b)| (a, b)).collect();
    let lo = Point::new(boxes.iter().map(|b| b.0.x).min()?, boxes.iter().map(|b| b.0.y).min()?);
    let hi = Point::new(boxes.iter().map(|b| b.1.x).max()?, boxes.iter().map(|b| b.1.y).max()?);
    Some((lo, hi))
}

/// Whether two components are one assembly part drawn on both documents: the same
/// occurrence, or, for a legacy component with none, the same part and label.
fn same_part(a: &Component, b: &Component) -> bool {
    match (&a.part, &b.part) {
        (Some(a_part), Some(b_part)) => match (&a_part.instance, &b_part.instance) {
            (Some(x), Some(y)) => x == y,
            (None, None) => a_part.key == b_part.key && a.reference == b.reference,
            _ => false,
        },
        _ => false,
    }
}

impl Document {
    /// Whether `wire` is a harness wire, by this document's kind.
    pub fn is_harness(&self, wire: &Wire) -> bool {
        Self::harness_in(self.kind, wire)
    }
    pub(crate) fn harness_in(kind: DocumentKind, wire: &Wire) -> bool {
        match kind {
            DocumentKind::Schematic => false,
            DocumentKind::Wiring => true,
            DocumentKind::Sheet => wire.harness,
        }
    }
    /// Whether `component` is on the board, by this document's kind.
    pub fn on_board(&self, component: &Component) -> bool {
        match self.kind {
            DocumentKind::Schematic => true,
            DocumentKind::Wiring => false,
            DocumentKind::Sheet => !component.off_board,
        }
    }
    /// Whether a net may reach `terminal`: on a sheet, only a pin of a part on the
    /// board. The refusal names the part.
    pub fn net_reaches(&self, terminal: &Terminal) -> Result<(), String> {
        if self.kind != DocumentKind::Sheet {
            return Ok(());
        }
        if let Some(z) = self.zone_member(terminal.component) {
            return Err(format!(
                "{} is inside the board {}, a document of its own, and its nets are drawn there. Connect to it with a \
                 harness wire on a point the board declares (Harness diagram).",
                self.terminal_name(terminal),
                z.reference
            ));
        }
        match self.components.iter().find(|c| c.id == terminal.component) {
            Some(c) if c.off_board => Err(format!(
                "{} is off the board, and a net works only inside a board zone. Connect it with a harness wire \
                 (Harness diagram), or drag it onto the board.",
                c.reference
            )),
            _ => Ok(()),
        }
    }
    /// Whether a harness wire may end on `terminal`: anywhere but a pin inside a
    /// sub-board the board declares no point for. The refusal names the pin and says
    /// what to do.
    pub fn harness_reaches(&self, terminal: &Terminal) -> Result<(), String> {
        match self.zone_member(terminal.component) {
            Some(z) if !z.declared.contains(terminal) => Err(format!(
                "{} is inside the board {}, which declares no point for it, and a harness wire ends inside a board \
                 only on a point it declares. Declare it on the board: a port point that maps to it (Qualify).",
                self.terminal_name(terminal),
                z.reference
            )),
            _ => Ok(()),
        }
    }
    /// The sub-board zone a part of the sheet is drawn from.
    pub fn zone_member(&self, id: Uuid) -> Option<&SheetZone> {
        self.zones.iter().find(|z| z.members.contains(&id))
    }
    /// The sub-board zone a net of the sheet is drawn from.
    pub fn zone_of_net(&self, id: Uuid) -> Option<&SheetZone> {
        self.zones.iter().find(|z| z.nets.contains(&id))
    }
    /// This sheet with its sub-board zones left out: its own board's parts, nets and
    /// markers, and the parts off any board. What its nets and its check read.
    pub fn without_zones(&self) -> Document {
        let mut own = self.clone();
        for z in &self.zones {
            own.components.retain(|c| !z.members.contains(&c.id));
            own.wires.retain(|w| !z.nets.contains(&w.id));
            own.labels.retain(|l| !z.labels.contains(&l.id));
            for j in &z.junctions {
                own.junctions.remove(j);
            }
        }
        own.zones.clear();
        let gone: BTreeSet<Uuid> = self.zones.iter().flat_map(|z| z.members.iter().copied()).collect();
        for w in &mut own.wires {
            for t in [&mut w.start, &mut w.end] {
                if t.as_ref().is_some_and(|t| gone.contains(&t.component)) {
                    *t = None;
                }
            }
        }
        own
    }
    /// The box round a sub-board zone's parts: `None` for one with none.
    pub fn sub_zone_extent(&self, occurrence: &str) -> Option<(Point, Point)> {
        let z = self.zones.iter().find(|z| z.occurrence == occurrence)?;
        let members: Vec<Component> =
            self.components.iter().filter(|c| z.members.contains(&c.id)).cloned().collect();
        extent_of(&members)
    }
    /// Move a sub-board zone, its parts and nets with it, and the harness wire ends
    /// on them.
    pub fn move_zone(&mut self, occurrence: &str, by: Point) -> Result<(), String> {
        let z = self
            .zones
            .iter_mut()
            .find(|z| z.occurrence == occurrence)
            .ok_or_else(|| format!("No sub-board zone {occurrence}"))?;
        z.at = z.at.offset(by);
        let (members, nets, labels) = (z.members.clone(), z.nets.clone(), z.labels.clone());
        let junctions: Vec<Point> = z.junctions.iter().map(|j| j.offset(by)).collect();
        for j in &z.junctions {
            self.junctions.remove(j);
        }
        z.junctions = junctions;
        self.junctions.extend(z.junctions.iter().copied());
        for l in self.labels.iter_mut().filter(|l| labels.contains(&l.id)) {
            l.at = l.at.offset(by);
        }
        for c in self.components.iter_mut().filter(|c| members.contains(&c.id)) {
            shift_component(c, by);
        }
        for w in self.wires.iter_mut().filter(|w| nets.contains(&w.id)) {
            shift_wire(w, by);
        }
        let touching: Vec<usize> = (0..self.wires.len())
            .filter(|i| {
                let w = &self.wires[*i];
                w.harness && [&w.start, &w.end].into_iter().flatten().any(|t| members.contains(&t.component))
            })
            .collect();
        for i in touching {
            let mut w = self.wires[i].clone();
            reattach(self, &mut w);
            self.wires[i] = w;
        }
        Ok(())
    }
    /// Why the edit from `before` to `self` is refused: it changed a sub-board's parts
    /// or nets other than by moving the whole zone. They are the board's own document,
    /// read-only on this sheet until the zone is entered (step 3).
    pub fn zone_edit_refusal(&self, before: &Document) -> Option<String> {
        if self.kind != DocumentKind::Sheet {
            return None;
        }
        for was in &before.zones {
            let refusal = || {
                Some(format!(
                    "{}'s parts and nets are read-only here: they are the board's own document. Move the whole zone, \
                     or edit the board in its own document.",
                    was.reference
                ))
            };
            let Some(now) = self.zones.iter().find(|z| z.occurrence == was.occurrence) else { return refusal() };
            if (&now.members, &now.nets, &now.declared, &now.labels) != (&was.members, &was.nets, &was.declared, &was.labels) {
                return refusal();
            }
            let by = Point::new(now.at.x - was.at.x, now.at.y - was.at.y);
            let moved: Vec<Point> = was.junctions.iter().map(|j| j.offset(by)).collect();
            if now.junctions != moved || now.junctions.iter().any(|j| !self.junctions.contains(j)) {
                return refusal();
            }
            for id in &was.labels {
                let (Some(a), Some(b)) =
                    (before.labels.iter().find(|l| l.id == *id), self.labels.iter().find(|l| l.id == *id))
                else {
                    return refusal();
                };
                let mut a = a.clone();
                a.at = a.at.offset(by);
                if a != *b {
                    return refusal();
                }
            }
            for id in &was.members {
                let (Some(a), Some(b)) = (
                    before.components.iter().find(|c| c.id == *id),
                    self.components.iter().find(|c| c.id == *id),
                ) else {
                    return refusal();
                };
                let mut a = a.clone();
                shift_component(&mut a, by);
                if a != *b {
                    return refusal();
                }
            }
            for id in &was.nets {
                let (Some(a), Some(b)) =
                    (before.wires.iter().find(|w| w.id == *id), self.wires.iter().find(|w| w.id == *id))
                else {
                    return refusal();
                };
                let mut a = a.clone();
                shift_wire(&mut a, by);
                if a != *b {
                    return refusal();
                }
            }
        }
        None
    }
    /// The parts on the board, as the zone drawn round them spans the sheet: `None`
    /// when none is.
    pub fn board_zone_extent(&self) -> Option<(Point, Point)> {
        if self.kind != DocumentKind::Sheet {
            return None;
        }
        let on: Vec<Component> = self.components.iter().filter(|c| !c.off_board).cloned().collect();
        extent_of(&on)
    }
    /// Put a part of a sheet on the board (`off_board == false`) or take it off.
    ///
    /// Taking it off lets go of everything of the board's that holds its pins (net
    /// wire ends, net flags, no-connect and power-flag markers); its harness wires
    /// stay. A part placed from an assembly part that has no pads is refused entry,
    /// with the reason, because the board could not seat it.
    pub fn set_off_board(&mut self, id: Uuid, off_board: bool) -> Result<(), String> {
        if self.kind != DocumentKind::Sheet {
            return Err("Only a sheet has parts off the board".into());
        }
        let c = self.components.iter().find(|c| c.id == id).ok_or("Component no longer exists")?;
        if let Some(z) = self.zone_member(id) {
            return Err(format!(
                "{} is a part of the board {}, a document of its own. To move it, delete it here and add it inside \
                 the board: enter the board's zone, then Add component.",
                c.reference, z.reference
            ));
        }
        if c.off_board == off_board {
            return Ok(());
        }
        if !off_board && c.part.is_some() && c.pads.is_none() {
            return Err(format!(
                "{} has no pads, so the board cannot seat it. Give its part pads in the Pads workbench first.",
                c.reference
            ));
        }
        if off_board {
            let mine = |t: &Option<Terminal>| t.as_ref().is_some_and(|t| t.component == id);
            for w in self.wires.iter_mut().filter(|w| !w.harness) {
                if mine(&w.start) {
                    w.start = None;
                }
                if mine(&w.end) {
                    w.end = None;
                }
            }
            for l in &mut self.labels {
                if l.terminal.as_ref().is_some_and(|t| t.component == id) {
                    l.terminal = None;
                    l.flag = true;
                }
            }
            self.no_connects.retain(|t| t.component != id);
            self.power_flags.retain(|t| t.component != id);
            self.board.placements.retain(|p| p.component != id);
        }
        self.components.iter_mut().find(|c| c.id == id).unwrap().off_board = off_board;
        Ok(())
    }
    /// A sheet's rules, past the ones every document keeps.
    pub(crate) fn validate_sheet(&self) -> Result<(), String> {
        let off: BTreeSet<Uuid> = self.components.iter().filter(|c| c.off_board).map(|c| c.id).collect();
        let reference = |t: &Terminal| self.terminal_name(t);
        let mut seen = BTreeSet::new();
        for z in &self.zones {
            if !seen.insert(&z.occurrence) {
                return Err(format!("The board {} has two zones", z.reference));
            }
            if z.members.iter().any(|id| !off.contains(id)) || z.nets.iter().any(|id| !self.wires.iter().any(|w| w.id == *id && !w.harness)) {
                return Err(format!("The zone of the board {} names a part or net the sheet does not hold as its own", z.reference));
            }
        }
        let mut ids = BTreeSet::new();
        let mut pins = BTreeSet::new();
        for w in &self.wires {
            if let Some(z) = self.zone_of_net(w.id) {
                if [&w.start, &w.end].into_iter().flatten().any(|t| !z.members.contains(&t.component)) {
                    return Err(format!("A net of the board {} leaves its zone", z.reference));
                }
                continue;
            }
            if w.harness {
                if w.connection_id.trim().is_empty() || !ids.insert(&w.connection_id) {
                    return Err("Connection IDs must be nonempty and unique".into());
                }
                let (Some(start), Some(end)) = (&w.start, &w.end) else {
                    return Err(format!("Connection {} must join two pins", w.connection_id));
                };
                for t in [start, end] {
                    if !pins.insert((t.component, &t.pin)) {
                        return Err(format!(
                            "{} has more than one harness wire; a pin takes one",
                            reference(t)
                        ));
                    }
                }
            } else {
                for t in [&w.start, &w.end].into_iter().flatten() {
                    if self.zone_member(t.component).is_some() {
                        self.net_reaches(t)?;
                    }
                    if off.contains(&t.component) {
                        return Err(format!(
                            "A net reaches {}, which is off the board; a net works only inside a board zone",
                            reference(t)
                        ));
                    }
                }
            }
        }
        let zone_labels: BTreeSet<Uuid> = self.zones.iter().flat_map(|z| z.labels.iter().copied()).collect();
        let marks = self
            .labels
            .iter()
            .filter(|l| !zone_labels.contains(&l.id))
            .filter_map(|l| l.terminal.as_ref())
            .chain(&self.no_connects)
            .chain(&self.power_flags);
        for t in marks {
            if off.contains(&t.component) {
                return Err(format!("{} is off the board and takes no net marker", reference(t)));
            }
        }
        if self.board.placements.iter().any(|p| off.contains(&p.component)) {
            return Err("A part off the board has a board placement".into());
        }
        Ok(())
    }

    /// Read a board's schematic and a wiring diagram into one sheet. `offset` is where
    /// the host last drew the diagram this session (the schematic origin, in diagram
    /// coordinates); the diagram's own [`Document::board_zone`] wins over it, and
    /// with neither the diagram is put to the left of the board.
    pub fn merge_sheet(
        pcb: Option<Document>,
        diagram: Option<Document>,
        offset: Option<Point>,
    ) -> Result<(Document, SheetSource), String> {
        Self::merge_sheet_with(pcb, diagram, offset, vec![])
    }
    /// [`Document::merge_sheet`] for an assembly that places boards of its own: each
    /// is a zone of the sheet, drawn from its schematic where the diagram's
    /// [`Document::zones`] say, else beside everything else.
    pub fn merge_sheet_with(
        pcb: Option<Document>,
        diagram: Option<Document>,
        offset: Option<Point>,
        boards: Vec<SubBoard>,
    ) -> Result<(Document, SheetSource), String> {
        if let Some(p) = &pcb
            && p.kind != DocumentKind::Schematic
        {
            return Err(format!("the PCB sheet holds a {:?} document, not a schematic", p.kind));
        }
        if let Some(d) = &diagram
            && d.kind != DocumentKind::Wiring
        {
            return Err(format!("the harness diagram holds a {:?} document, not a wiring diagram", d.kind));
        }
        let offset = diagram.as_ref().and_then(|d| d.board_zone).or(offset).unwrap_or_else(|| {
            match (
                pcb.as_ref().and_then(|p| extent_of(&p.components)),
                diagram.as_ref().and_then(|d| extent_of(&d.components)),
            ) {
                (Some((board_lo, _)), Some((diagram_lo, diagram_hi))) => Point::new(
                    diagram_hi.x + SIDE_BY_SIDE_GAP - board_lo.x,
                    diagram_lo.y - board_lo.y,
                ),
                _ => Point::default(),
            }
        });
        let mut sheet = Document::new(DocumentKind::Sheet);
        if let Some(p) = pcb.as_ref() {
            sheet.title = p.title.clone();
            sheet.components = p.components.clone();
            sheet.wires = p.wires.clone();
            sheet.junctions = p.junctions.clone();
            sheet.labels = p.labels.clone();
            sheet.board = p.board.clone();
            sheet.no_connects = p.no_connects.clone();
            sheet.power_flags = p.power_flags.clone();
        } else if let Some(d) = diagram.as_ref() {
            sheet.title = d.title.clone();
        }
        // The sub-board zones, where the diagram drew them, else beside everything else.
        let mut drawn: Vec<Component> = sheet.components.clone();
        if let Some(d) = diagram.as_ref() {
            drawn.extend(d.components.iter().map(|c| {
                let mut c = c.clone();
                shift_component(&mut c, neg(offset));
                c
            }));
        }
        for b in &boards {
            if b.schematic.kind != DocumentKind::Schematic {
                return Err(format!("the board {} holds a {:?} document, not a schematic", b.reference, b.schematic.kind));
            }
            let occ = &b.occurrence;
            let stored = diagram
                .as_ref()
                .and_then(|d| d.zones.iter().find(|z| z.occurrence == *occ))
                .map(|z| z.at.offset(neg(offset)));
            let at = stored.unwrap_or_else(|| match (extent_of(&drawn), extent_of(&b.schematic.components)) {
                (Some((lo, hi)), Some((inner_lo, _))) => {
                    Point::new(hi.x + SIDE_BY_SIDE_GAP - inner_lo.x, lo.y - inner_lo.y)
                }
                _ => Point::default(),
            });
            let mut zone = SheetZone { occurrence: occ.clone(), at, reference: b.reference.clone(), ..SheetZone::default() };
            for c in &b.schematic.components {
                let mut c = c.clone();
                c.id = zone_id(occ, c.id);
                c.reference = format!("{}/{}", b.reference, c.reference);
                if let Some(part) = &mut c.part
                    && let Some(inner) = &part.instance
                {
                    part.instance = Some(format!("{occ}:{inner}"));
                }
                c.off_board = true;
                shift_component(&mut c, at);
                zone.members.push(c.id);
                drawn.push(c.clone());
                sheet.components.push(c);
            }
            for w in &b.schematic.wires {
                let mut w = w.clone();
                w.id = zone_id(occ, w.id);
                for t in [&mut w.start, &mut w.end].into_iter().flatten() {
                    t.component = zone_id(occ, t.component);
                }
                shift_wire(&mut w, at);
                w.harness = false;
                zone.nets.push(w.id);
                sheet.wires.push(w);
            }
            for l in &b.schematic.labels {
                let mut l = l.clone();
                l.id = zone_id(occ, l.id);
                l.at = l.at.offset(at);
                if let Some(t) = &mut l.terminal {
                    t.component = zone_id(occ, t.component);
                }
                zone.labels.push(l.id);
                sheet.labels.push(l);
            }
            for j in &b.schematic.junctions {
                let j = j.offset(at);
                if sheet.junctions.insert(j) {
                    zone.junctions.push(j);
                }
            }
            zone.declared = b
                .declared
                .iter()
                .map(|t| Terminal { component: zone_id(occ, t.component), pin: t.pin.clone() })
                .collect();
            sheet.zones.push(zone);
        }
        let candidates: Vec<Component> = sheet.components.iter().filter(|c| !c.off_board || sheet.zone_member(c.id).is_some()).cloned().collect();
        let mut twins = BTreeMap::new();
        if let Some(d) = diagram.as_ref() {
            // The diagram's id for a part → the sheet's.
            let mut renamed: BTreeMap<Uuid, Uuid> = BTreeMap::new();
            let mut taken: BTreeSet<Uuid> = BTreeSet::new();
            for c in &d.components {
                let twin = candidates.iter().find(|b| !taken.contains(&b.id) && (b.id == c.id || same_part(b, c)));
                match twin {
                    Some(b) => {
                        taken.insert(b.id);
                        renamed.insert(c.id, b.id);
                        twins.insert(b.id, c.clone());
                    }
                    None => {
                        let mut c = c.clone();
                        shift_component(&mut c, neg(offset));
                        c.off_board = true;
                        sheet.components.push(c);
                    }
                }
            }
            for w in &d.wires {
                let mut w = w.clone();
                shift_wire(&mut w, neg(offset));
                for t in [&mut w.start, &mut w.end].into_iter().flatten() {
                    if let Some(id) = renamed.get(&t.component) {
                        t.component = *id;
                    }
                }
                w.harness = true;
                sheet.wires.push(w);
            }
        }
        let mut moved_wires = BTreeMap::new();
        if let Some(d) = diagram.as_ref() {
            let first = sheet.wires.len() - d.wires.len();
            for (i, original) in d.wires.iter().enumerate() {
                let mut w = sheet.wires[first + i].clone();
                reattach(&sheet, &mut w);
                if w != sheet.wires[first + i] {
                    moved_wires.insert(w.id, (original.clone(), w.clone()));
                    sheet.wires[first + i] = w;
                }
            }
        }
        sheet.validate()?;
        let zones = sheet.zones.clone();
        Ok((sheet, SheetSource { pcb, diagram, offset, twins, moved_wires, zones }))
    }

    /// Write a sheet back as a board's schematic and a wiring diagram, each one EQUAL
    /// to the one [`SheetSource`] merged where nothing of it was edited.
    ///
    /// - The schematic holds the parts on the board, the nets and the layout; it is
    ///   `None` when there was none and the sheet has nothing of the board's.
    /// - The diagram holds the parts off the board, the harness wires, and a twin of
    ///   each part on the board that a harness wire reaches (or that had one).
    pub fn split_sheet(&self, source: &SheetSource) -> Result<SplitSheet, String> {
        if self.kind != DocumentKind::Sheet {
            return Err("only a sheet splits".into());
        }
        let by_id: BTreeMap<Uuid, &Component> = self.components.iter().map(|c| (c.id, c)).collect();
        let order = |base: Option<&Document>, wanted: &dyn Fn(&Component) -> bool| -> Vec<Uuid> {
            let mut ids: Vec<Uuid> = base
                .into_iter()
                .flat_map(|d| d.components.iter().map(|c| c.id))
                .filter(|id| by_id.get(id).is_some_and(|c| wanted(c)))
                .collect();
            for c in &self.components {
                if wanted(c) && !ids.contains(&c.id) {
                    ids.push(c.id);
                }
            }
            ids
        };

        // The schematic.
        let on_ids = order(source.pcb.as_ref(), &|c: &Component| !c.off_board);
        let nets: Vec<Wire> =
            self.wires.iter().filter(|w| !w.harness && self.zone_of_net(w.id).is_none()).cloned().collect();
        let zone_labels: BTreeSet<Uuid> = self.zones.iter().flat_map(|z| z.labels.iter().copied()).collect();
        let zone_junctions: BTreeSet<Point> = self.zones.iter().flat_map(|z| z.junctions.iter().copied()).collect();
        let labels: Vec<crate::Label> = self.labels.iter().filter(|l| !zone_labels.contains(&l.id)).cloned().collect();
        let junctions: BTreeSet<Point> = self.junctions.difference(&zone_junctions).copied().collect();
        let board_has_content = source.pcb.is_some()
            || !on_ids.is_empty()
            || !nets.is_empty()
            || !labels.is_empty()
            || !junctions.is_empty()
            || self.board != crate::board::Board::default();
        let pcb = board_has_content.then(|| {
            let mut p = source.pcb.clone().unwrap_or_else(|| Document::new(DocumentKind::Schematic));
            p.components = on_ids.iter().map(|id| by_id[id].clone()).collect();
            p.wires = nets;
            p.junctions = junctions;
            p.labels = labels;
            p.board = self.board.clone();
            p.board.placements.retain(|pl| on_ids.contains(&pl.component));
            p.no_connects = self.no_connects.clone();
            p.power_flags = self.power_flags.clone();
            p
        });

        // The diagram.
        let harness: Vec<&Wire> = self.wires.iter().filter(|w| w.harness).collect();
        let reached: BTreeSet<Uuid> = harness
            .iter()
            .flat_map(|w| [&w.start, &w.end])
            .flatten()
            .map(|t| t.component)
            .collect();
        // The sheet's id → the diagram's id, for every part the diagram holds.
        let mut diagram_id: BTreeMap<Uuid, Uuid> = BTreeMap::new();
        let mut components: Vec<Component> = Vec::new();
        let offset = source.offset;
        let pcb_base = |id: &Uuid| source.pcb.as_ref().and_then(|p| p.components.iter().find(|c| c.id == *id));
        // A part on the board, as its twin writes it: the twin as merged while the
        // part is the one merged; else the part, where the twin was or where it is.
        let twin_of = |c: &Component, twin: Option<&Component>| -> Component {
            match twin {
                Some(t) if pcb_base(&c.id) == Some(c) => t.clone(),
                Some(t) => {
                    let mut out = c.clone();
                    out.id = t.id;
                    out.off_board = false;
                    if c.symbol == t.symbol {
                        out.at = t.at;
                        out.rotation = t.rotation;
                        out.gates = t.gates.clone();
                    } else {
                        shift_component(&mut out, offset);
                    }
                    out
                }
                None => {
                    let mut out = c.clone();
                    out.off_board = false;
                    shift_component(&mut out, offset);
                    out
                }
            }
        };
        let emit = |c: &Component, components: &mut Vec<Component>, diagram_id: &mut BTreeMap<Uuid, Uuid>| {
            let twin = source.twins.get(&c.id);
            // A sub-board's part, as the diagram keeps it to land harness wires on:
            // the copy it had, else the part where the sheet draws it.
            let member = self.zone_member(c.id).is_some();
            let out = if member && let Some(t) = twin {
                t.clone()
            } else if c.off_board {
                let mut out = c.clone();
                out.off_board = false;
                shift_component(&mut out, offset);
                // A part the board had, and its twin's id still the diagram's.
                if let Some(t) = twin {
                    out.id = t.id;
                }
                out
            } else {
                twin_of(c, twin)
            };
            diagram_id.insert(c.id, out.id);
            components.push(out);
        };
        let wants_diagram = |c: &Component| {
            (c.off_board && self.zone_member(c.id).is_none())
                || reached.contains(&c.id)
                || source.twins.contains_key(&c.id)
        };
        // The diagram's own order first: a twin by the sheet id it merged into, any
        // other part by its own id.
        let twin_sheet_id: BTreeMap<Uuid, Uuid> = source.twins.iter().map(|(sheet, t)| (t.id, *sheet)).collect();
        if let Some(d) = source.diagram.as_ref() {
            for base in &d.components {
                let sheet_id = twin_sheet_id.get(&base.id).copied().unwrap_or(base.id);
                if let Some(c) = by_id.get(&sheet_id)
                    && wants_diagram(c)
                    && !diagram_id.contains_key(&sheet_id)
                {
                    emit(c, &mut components, &mut diagram_id);
                }
            }
        }
        for c in &self.components {
            if wants_diagram(c) && !diagram_id.contains_key(&c.id) {
                emit(c, &mut components, &mut diagram_id);
            }
        }
        // Ends land where the diagram's copy of each part is.
        let mut placed = Document::new(DocumentKind::Wiring);
        placed.components = components.clone();
        let wires: Vec<Wire> = harness
            .iter()
            .map(|w| {
                // An unedited wire whose ends the merge moved goes back as the
                // diagram had it, while the parts it joins are where they were.
                if let Some((original, merged)) = source.moved_wires.get(&w.id)
                    && merged == *w
                {
                    let mut kept = original.clone();
                    reattach(&placed, &mut kept);
                    if kept == *original {
                        return kept;
                    }
                }
                let mut w = (*w).clone();
                w.harness = false;
                shift_wire(&mut w, offset);
                for t in [&mut w.start, &mut w.end].into_iter().flatten() {
                    if let Some(id) = diagram_id.get(&t.component) {
                        t.component = *id;
                    }
                }
                reattach(&placed, &mut w);
                w
            })
            .collect();
        // Where each sub-board zone sits, written once anything of the diagram's or a
        // zone moved, so an open writes nothing.
        // A stored place for a board the assembly no longer places is dead: it is
        // dropped with the first write of this sheet, a board edit included, so a
        // saved file carries no dead placement. An open alone writes nothing.
        let dead = source.diagram.as_ref().is_some_and(|d| {
            d.zones.iter().any(|z| !self.zones.iter().any(|live| live.occurrence == z.occurrence))
        });
        let zones_moved = self.zones.len() != source.zones.len()
            || self.zones.iter().zip(&source.zones).any(|(a, b)| (&a.occurrence, a.at) != (&b.occurrence, b.at))
            || (dead && pcb != source.pcb);
        let diagram = (source.diagram.is_some() || !components.is_empty() || !wires.is_empty() || zones_moved).then(|| {
            let mut d = source.diagram.clone().unwrap_or_else(|| Document::new(DocumentKind::Wiring));
            let unchanged = source.diagram.as_ref().is_some_and(|base| base.components == components && base.wires == wires);
            d.components = components;
            d.wires = wires;
            if (!unchanged || zones_moved) && (offset != Point::default() || d.board_zone.is_some()) {
                d.board_zone = Some(offset);
            }
            if !unchanged || zones_moved {
                d.zones = self
                    .zones
                    .iter()
                    .map(|z| SheetZone { occurrence: z.occurrence.clone(), at: z.at.offset(offset), ..SheetZone::default() })
                    .collect();
            }
            d
        });
        Ok(SplitSheet { pcb, diagram, diagram_ids: diagram_id })
    }
}

