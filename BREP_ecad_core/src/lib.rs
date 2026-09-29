//! UI-independent schematic and board documents, symbol import, and connectivity.
//!
//! Object IDs are random version 4 UUIDs. On `wasm32-unknown-unknown` the randomness
//! comes from the host: enable the `uuid` crate's `js` feature (as `BREP_app` does) or
//! configure another `getrandom` backend, or creating an object fails.
pub mod autoroute;
pub mod board;
pub mod erc;
pub mod fabrication;
pub mod footprint;
pub mod kicad;
pub mod netlist;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
pub use uuid::Uuid;

/// Integer micrometres; positive Y points down on a schematic sheet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}
impl Point {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
    pub fn offset(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }
    pub fn rotate(self, turns: u8) -> Self {
        match turns % 4 {
            1 => Self::new(-self.y, self.x),
            2 => Self::new(-self.x, -self.y),
            3 => Self::new(self.y, -self.x),
            _ => self,
        }
    }
    pub fn snapped(self) -> Self {
        Self::new(
            (self.x as f64 / 1270.).round() as i32 * 1270,
            (self.y as f64 / 1270.).round() as i32 * 1270,
        )
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Graphic {
    Path(Vec<Point>),
    Circle { center: Point, radius: i32 },
    Text { at: Point, text: String, size: i32 },
}
fn unit_is_unset(unit: &u32) -> bool {
    *unit == 0
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pin {
    #[serde(default)]
    pub hidden: bool,
    /// The symbol UNIT this pin belongs to, 1-based, as KiCad tags one. `0`
    /// means unassigned and reads as unit 1. A BREP part maps one unit to one
    /// port group, so this is what says which connector a pin is on
    /// (`BREP_kernel/src/feature_pipeline/part_pins.rs`).
    ///
    /// Omitted when unset, so a SINGLE-UNIT symbol serializes exactly as it did
    /// before units existed. The editor compares its held symbol against the
    /// document's block byte for byte to know whether it is in step, and a
    /// field that always wrote itself would put every stored symbol out of step
    /// until it was saved again.
    #[serde(default, skip_serializing_if = "unit_is_unset")]
    pub unit: u32,
    /// The GATE this pin is drawn with, 1-based: one of the separately placed
    /// sections of a device, such as the two amplifiers and the power section of
    /// a dual op-amp (KiCad calls them units and a sheet names them `U1A`, `U1B`,
    /// …). `0` means the symbol is not split into gates.
    ///
    /// A gate is a DRAWING matter only. Every gate of a device is one package,
    /// one footprint and one port group, and a pin still binds by its number.
    /// That is what keeps it apart from [`Pin::unit`], which says which port
    /// GROUP (which connector) a pin is on: the LM358's pin 5 is on gate B and in
    /// the same group as pin 1. Omitted when unset, for the reason `unit` is.
    #[serde(default, skip_serializing_if = "unit_is_unset")]
    pub gate: u32,
    pub number: String,
    pub name: String,
    pub electrical_type: String,
    pub at: Point,
    pub end: Point,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Symbol {
    #[serde(default)]
    pub unit_count: u32,
    #[serde(default)]
    pub properties: BTreeMap<String, String>,
    #[serde(default)]
    pub power_net: Option<String>,
    pub library_id: String,
    pub reference_prefix: String,
    pub description: String,
    pub graphics: Vec<Graphic>,
    pub pins: Vec<Pin>,
    /// The gate ([`Pin::gate`]) each of [`Symbol::graphics`] is drawn with, by
    /// index. A graphic past the end of this list, or tagged `0`, belongs to no
    /// gate and is drawn where the component itself is. Empty for a symbol not
    /// split into gates, and then omitted.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub graphic_gates: Vec<u32>,
    /// KiCad's symbol-level `(pin_names (hide yes))`: the pins keep their names,
    /// and the sheet does not print them. A connector's `Pin_1` … `Pin_4` or a
    /// resistor's `~` say nothing the pin number does not.
    #[serde(default, skip_serializing_if = "is_false")]
    pub hide_pin_names: bool,
    /// KiCad's symbol-level `(pin_numbers (hide yes))`, as a resistor or a
    /// capacitor carries it.
    #[serde(default, skip_serializing_if = "is_false")]
    pub hide_pin_numbers: bool,
}
fn is_false(value: &bool) -> bool {
    !*value
}
impl Symbol {
    /// The value a part placed from this symbol starts with: KiCad's `Value`
    /// property when the symbol carries one (`R`, `LM358`, `Conn_01x04`), else
    /// the part of its library id after the `:`, which is what every component
    /// was given before the property was read. The user's value (`10k`) is typed
    /// over it on the sheet; see [`Document::set_value`].
    pub fn default_value(&self) -> String {
        match self.properties.get("Value").map(|v| v.trim()).filter(|v| !v.is_empty()) {
            Some(value) => value.to_owned(),
            None => self.library_id.split(':').next_back().unwrap_or("").to_owned(),
        }
    }
    /// How many gates the symbol is split into; `0` or `1` is a symbol drawn
    /// whole.
    pub fn gate_count(&self) -> u32 {
        self.pins
            .iter()
            .map(|p| p.gate)
            .chain(self.graphic_gates.iter().copied())
            .max()
            .unwrap_or(0)
    }
    /// Bring a multi-unit symbol written by the KiCad import BEFORE gates
    /// existed up to the gate shape, as a fresh import would write it now.
    /// Returns whether it changed anything.
    ///
    /// That import put each KiCad unit into [`Pin::unit`], which is the PORT
    /// GROUP a pin binds into, set `unit_count` to the number of units, and
    /// wrote a `Unit N` caption after each unit's graphics. So an LM358 bound 3
    /// of its 8 pins and was placed as one block. It is recognised by exactly
    /// those captions: one `Graphic::Text { text: "Unit N", size: 1270 }` at
    /// `y == -15000` for every N in 1..=`unit_count`, in order, with every pin's
    /// unit in that range and no gate tagged yet. Nothing else wrote them, and
    /// a part split into connectors on purpose (two headers, `Pin::unit` 1 and
    /// 2) has no such captions and is left alone.
    ///
    /// Each unit becomes a gate: `Pin::gate` takes the unit and `Pin::unit`
    /// goes back to `0`, a graphic takes the gate of the caption that closes
    /// its run, the captions go (a sheet names each gate `U1A` itself), and
    /// `unit_count` becomes 1. Coordinates are not touched.
    pub fn migrate_legacy_units(&mut self) -> bool {
        let Some(captions) = self.legacy_unit_captions() else {
            return false;
        };
        let mut gates = Vec::with_capacity(self.graphics.len());
        let mut gate = 1;
        for (index, _) in self.graphics.iter().enumerate() {
            if captions.contains(&index) {
                gate += 1;
            } else {
                // Nothing the old import wrote follows the last caption; if
                // something does, it is drawn with the part, as before.
                gates.push(if gate <= self.unit_count { gate } else { 0 });
            }
        }
        let mut index = 0;
        self.graphics.retain(|_| {
            index += 1;
            !captions.contains(&(index - 1))
        });
        self.graphic_gates = gates;
        for pin in &mut self.pins {
            pin.gate = pin.unit;
            pin.unit = 0;
        }
        self.unit_count = 1;
        true
    }
    /// The indices of the `Unit 1` … `Unit N` captions of a symbol in the
    /// pre-gate import's shape ([`Symbol::migrate_legacy_units`]), or `None`
    /// when it is not in that shape.
    fn legacy_unit_captions(&self) -> Option<Vec<usize>> {
        let count = self.unit_count;
        if count < 2 || self.gate_count() > 0 {
            return None;
        }
        if !self.pins.iter().all(|p| (1..=count).contains(&p.unit)) {
            return None;
        }
        let captions: Vec<(usize, u32)> = self
            .graphics
            .iter()
            .enumerate()
            .filter_map(|(i, g)| match g {
                Graphic::Text { at, text, size } if *size == 1270 && at.y == -15000 => text
                    .strip_prefix("Unit ")
                    .and_then(|n| n.parse::<u32>().ok())
                    .map(|n| (i, n)),
                _ => None,
            })
            .collect();
        let in_order = captions.iter().map(|&(_, n)| n).eq(1..=count);
        in_order.then(|| captions.into_iter().map(|(i, _)| i).collect())
    }
    /// The gate graphic `index` is drawn with; `0` for none.
    pub fn graphic_gate(&self, index: usize) -> u32 {
        self.graphic_gates.get(index).copied().unwrap_or(0)
    }
    /// The box gate `gate`'s pins and graphics span in symbol coordinates, or
    /// `None` when nothing is drawn with it.
    pub fn gate_extent(&self, gate: u32) -> Option<(Point, Point)> {
        let mut points = vec![];
        for pin in self.pins.iter().filter(|p| p.gate == gate) {
            points.extend([pin.at, pin.end]);
        }
        for (i, g) in self.graphics.iter().enumerate() {
            if self.graphic_gate(i) == gate {
                points.extend(graphic_points(g));
            }
        }
        extent(&points)
    }
}
/// The letter a sheet adds to a reference for gate `gate`: `A` for 1, `B` for
/// 2, …, `Z`, then `AA`, as KiCad names units.
pub fn gate_letter(gate: u32) -> String {
    let mut n = gate.max(1);
    let mut letters = vec![];
    while n > 0 {
        n -= 1;
        letters.push(char::from(b'A' + (n % 26) as u8));
        n /= 26;
    }
    letters.iter().rev().collect()
}
fn graphic_points(g: &Graphic) -> Vec<Point> {
    match g {
        Graphic::Path(path) => path.clone(),
        Graphic::Circle { center, radius } => vec![
            Point::new(center.x - radius, center.y - radius),
            Point::new(center.x + radius, center.y + radius),
        ],
        Graphic::Text { at, .. } => vec![*at],
    }
}
fn extent(points: &[Point]) -> Option<(Point, Point)> {
    let lo = Point::new(points.iter().map(|p| p.x).min()?, points.iter().map(|p| p.y).min()?);
    let hi = Point::new(points.iter().map(|p| p.x).max()?, points.iter().map(|p| p.y).max()?);
    Some((lo, hi))
}
/// The part a component was placed from: the host's key for that part, and the
/// signature of the version whose symbol and pads were copied. A host compares the
/// signature against the part as it stands now to find components to refresh.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct PartSource {
    /// Opaque native assembly occurrence identity, independent of its display label.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub instance: Option<String>,
    pub key: String,
    pub signature: String,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Component {
    pub id: Uuid,
    /// The label this placed part goes by, such as `J1`. A host that keeps its own
    /// labels sets it with [`Document::set_reference`].
    pub reference: String,
    pub value: String,
    pub symbol: Symbol,
    pub at: Point,
    pub rotation: u8,
    /// The part this was placed from, absent for a component drawn without one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub part: Option<PartSource>,
    /// The part's board pads, copied when it was placed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pads: Option<board::Footprint>,
    /// Where each gate of a symbol split into gates ([`Pin::gate`]) sits on the
    /// sheet, one entry per gate: each is moved and turned on its own and named
    /// by the reference and its letter (`U1A`), while the component stays ONE
    /// part with one reference, one set of pads and one board placement. Empty,
    /// and omitted, for a symbol drawn whole.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub gates: Vec<GatePlacement>,
}
/// One gate of a component on the sheet: the gate's geometry, taken about
/// `pivot` in symbol coordinates, turned by `rotation` quarter turns and put
/// at `at`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GatePlacement {
    pub gate: u32,
    pub pivot: Point,
    pub at: Point,
    #[serde(default)]
    pub rotation: u8,
}
impl Component {
    /// Whether this is a part with a value of its own: not a power symbol or flag
    /// (a `#` reference, as KiCad's `#PWR` and `#FLG`, which name a net and are left
    /// out of the netlist and the BOM), and not a pinless placeholder.
    pub fn is_valued_part(&self) -> bool {
        self.symbol.power_net.is_none() && !self.reference.starts_with('#') && !self.symbol.pins.is_empty()
    }
    pub fn transform(&self, p: Point) -> Point {
        p.rotate(self.rotation).offset(self.at)
    }
    /// Where a symbol point drawn with gate `gate` lands on the sheet: through
    /// that gate's own placement when the component has one, else through the
    /// component's.
    pub fn gate_transform(&self, gate: u32, p: Point) -> Point {
        match self.gate(gate) {
            Some(g) => Point::new(p.x - g.pivot.x, p.y - g.pivot.y)
                .rotate(g.rotation)
                .offset(g.at),
            None => self.transform(p),
        }
    }
    /// The placement of gate `gate`, when the component places its gates apart.
    pub fn gate(&self, gate: u32) -> Option<&GatePlacement> {
        (gate > 0).then(|| self.gates.iter().find(|g| g.gate == gate)).flatten()
    }
    /// A pin's terminal on the sheet.
    pub fn pin_at(&self, pin: &Pin) -> Point {
        self.gate_transform(pin.gate, pin.at)
    }
    /// A pin's body end on the sheet.
    pub fn pin_end(&self, pin: &Pin) -> Point {
        self.gate_transform(pin.gate, pin.end)
    }
    /// Where a point of graphic `index` lands on the sheet.
    pub fn graphic_transform(&self, index: usize, p: Point) -> Point {
        self.gate_transform(self.symbol.graphic_gate(index), p)
    }
    /// The name a gate goes by on the sheet: `U1A` for gate 1 of `U1`; the
    /// reference alone for a component drawn whole.
    pub fn gate_reference(&self, gate: u32) -> String {
        if self.gates.is_empty() || gate == 0 {
            self.reference.clone()
        } else {
            format!("{}{}", self.reference, gate_letter(gate))
        }
    }
    /// The boxes the component covers on the sheet, one for each gate placed
    /// apart, else one for the whole symbol.
    pub fn sheet_boxes(&self) -> Vec<(u32, Point, Point)> {
        let gates: Vec<u32> = if self.gates.is_empty() {
            vec![0]
        } else {
            self.gates.iter().map(|g| g.gate).collect()
        };
        gates
            .into_iter()
            .map(|gate| {
                let mut points = vec![];
                for pin in &self.symbol.pins {
                    if self.gates.is_empty() || pin.gate == gate {
                        points.extend([self.pin_at(pin), self.pin_end(pin)]);
                    }
                }
                for (i, g) in self.symbol.graphics.iter().enumerate() {
                    if self.gates.is_empty() || self.symbol.graphic_gate(i) == gate {
                        points.extend(graphic_points(g).into_iter().map(|p| self.graphic_transform(i, p)));
                    }
                }
                let at = self.gate(gate).map_or(self.at, |g| g.at);
                points.push(at);
                let (lo, hi) = extent(&points).unwrap_or((at, at));
                (gate, lo, hi)
            })
            .collect()
    }
    /// Give the component one [`GatePlacement`] per gate of its symbol, keeping
    /// those it has. A new gate starts exactly where the symbol drawn whole puts
    /// it, so splitting moves nothing; a symbol drawn whole has none.
    pub fn sync_gates(&mut self) {
        let count = self.symbol.gate_count();
        if count <= 1 {
            self.gates.clear();
            return;
        }
        self.gates.retain(|g| (1..=count).contains(&g.gate));
        for gate in 1..=count {
            if self.gates.iter().any(|g| g.gate == gate) {
                continue;
            }
            let Some((lo, hi)) = self.symbol.gate_extent(gate) else {
                continue;
            };
            let pivot = Point::new((lo.x + hi.x) / 2, (lo.y + hi.y) / 2).snapped();
            self.gates.push(GatePlacement {
                gate,
                pivot,
                at: self.transform(pivot),
                rotation: self.rotation,
            });
        }
        self.gates.sort_by_key(|g| g.gate);
    }
}
/// A stable reference to a component terminal, independent of drawing position.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Terminal {
    pub component: Uuid,
    pub pin: String,
}
/// Interactive connection targets. Junctions use their exact sheet coordinates.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConnectionTarget {
    Terminal(Terminal),
    Junction(Point),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Wire {
    pub id: Uuid,
    pub a: Point,
    pub b: Point,
    #[serde(default)]
    pub bends: Vec<Point>,
    #[serde(default)]
    pub start: Option<Terminal>,
    #[serde(default)]
    pub end: Option<Terminal>,
    /// Wiring diagrams only: the connection's identifier in the connection list.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub connection_id: String,
    /// Wiring diagrams only: stock part number of the wire or cable.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub stock_part_number: String,
}
impl Wire {
    pub fn new(a: Point, b: Point) -> Self {
        Self {
            id: Uuid::new_v4(),
            a,
            b,
            bends: vec![],
            start: None,
            end: None,
            connection_id: String::new(),
            stock_part_number: String::new(),
        }
    }
    pub fn points(&self) -> Vec<Point> {
        std::iter::once(self.a)
            .chain(self.bends.iter().copied())
            .chain(std::iter::once(self.b))
            .collect()
    }
    pub fn set_points(&mut self, points: Vec<Point>) {
        let mut clean: Vec<Point> = vec![];
        for p in points {
            if clean.last() == Some(&p) {
                continue;
            }
            while clean.len() >= 2 {
                let a = clean[clean.len() - 2];
                let b = clean[clean.len() - 1];
                if (a.x == b.x && b.x == p.x) || (a.y == b.y && b.y == p.y) {
                    clean.pop();
                } else {
                    break;
                }
            }
            clean.push(p);
        }
        if clean.len() >= 2 {
            self.a = clean[0];
            self.b = *clean.last().unwrap();
            self.bends = clean[1..clean.len() - 1].to_vec();
        }
    }
    /// Move a segment perpendicular to itself. Outer terminals remain fixed.
    pub fn slide_segment(&mut self, index: usize, delta: Point) {
        let mut points = self.points();
        if index + 1 >= points.len() {
            return;
        }
        let a = points[index];
        let b = points[index + 1];
        let shift = if a.y == b.y {
            Point::new(0, delta.y)
        } else {
            Point::new(delta.x, 0)
        };
        if shift == Point::default() {
            return;
        }
        let moved_a = a.offset(shift);
        let moved_b = b.offset(shift);
        if index == 0 {
            points.insert(1, moved_a);
            points[2] = moved_b;
            if points.len() == 3 {
                points.push(b);
            }
        } else if index + 1 == points.len() - 1 {
            points[index] = moved_a;
            points.insert(index + 1, moved_b);
        } else {
            points[index] = moved_a;
            points[index + 1] = moved_b;
        }
        self.set_points(points);
    }
    /// Insert an offset in the middle of the selected segment.
    pub fn add_jog(&mut self, index: usize) {
        let mut points = self.points();
        if index + 1 >= points.len() {
            return;
        }
        let a = points[index];
        let b = points[index + 1];
        let p = Point::new(a.x + (b.x - a.x) / 3, a.y + (b.y - a.y) / 3);
        let q = Point::new(a.x + 2 * (b.x - a.x) / 3, a.y + 2 * (b.y - a.y) / 3);
        if a == p || p == q || q == b {
            return;
        }
        let offset = if a.y == b.y {
            Point::new(0, 2540)
        } else {
            Point::new(2540, 0)
        };
        points.splice(
            index + 1..index + 1,
            [p, p.offset(offset), q.offset(offset), q],
        );
        self.set_points(points);
    }
    pub fn removable_jog(&self, near: usize) -> Option<usize> {
        self.points()
            .windows(4)
            .enumerate()
            .filter(|(_, p)| p[0] != p[3] && (p[0].x == p[3].x || p[0].y == p[3].y))
            .min_by_key(|(i, _)| (i + 1).abs_diff(near))
            .map(|(i, _)| i)
    }
    pub fn remove_jog(&mut self, near: usize) {
        if let Some(i) = self.removable_jog(near) {
            let mut p = self.points();
            p.drain(i + 1..i + 3);
            self.set_points(p);
        }
    }
    fn follow_start(&mut self, at: Point) {
        if at == self.a {
            return;
        }
        let mut points = self.points();
        let horizontal = points[0].y == points[1].y;
        points[0] = at;
        if points.len() == 2 {
            let b = points[1];
            let bend = if horizontal {
                Point::new(b.x, at.y)
            } else {
                Point::new(at.x, b.y)
            };
            points.insert(1, bend);
        } else if horizontal {
            points[1].y = at.y;
        } else {
            points[1].x = at.x;
        }
        self.set_points(points);
    }
    pub fn follow(&mut self, a: Point, b: Point) {
        if a == b {
            self.set_points(vec![
                a,
                a.offset(Point::new(2540, 0)),
                a.offset(Point::new(2540, 2540)),
                a.offset(Point::new(0, 2540)),
                b,
            ]);
            return;
        }
        self.follow_start(a);
        let mut points = self.points();
        points.reverse();
        self.set_points(points);
        self.follow_start(b);
        let mut points = self.points();
        points.reverse();
        self.set_points(points);
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Label {
    #[serde(default)]
    pub flag: bool,
    #[serde(default)]
    pub rotation: u8,
    #[serde(default)]
    pub terminal: Option<Terminal>,
    pub id: Uuid,
    pub name: String,
    pub at: Point,
}
/// What a document is drawn for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DocumentKind {
    /// Wires, junctions, and labels form nets, which the board is laid out from.
    #[default]
    Schematic,
    /// Point-to-point connections without nets: every wire joins two symbol pins, each
    /// pin takes at most one wire, and the output is a connection list.
    Wiring,
}
impl DocumentKind {
    fn is_schematic(&self) -> bool {
        *self == Self::Schematic
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Document {
    pub version: u32,
    #[serde(default, skip_serializing_if = "DocumentKind::is_schematic")]
    pub kind: DocumentKind,
    pub title: String,
    pub components: Vec<Component>,
    pub wires: Vec<Wire>,
    pub junctions: BTreeSet<Point>,
    pub labels: Vec<Label>,
    /// Physical layout. Absent in version 1-3 documents.
    #[serde(default)]
    pub board: board::Board,
    /// Pins the user marked as meant to be left open: the electrical rule check
    /// ([`Document::erc`]) does not ask for them to be connected. Omitted when
    /// empty, so a document without markers reads as it did before they existed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub no_connects: Vec<Terminal>,
    /// Pins carrying a power flag, KiCad's PWR_FLAG: the net they are on is driven
    /// from off the sheet, so its power inputs are not reported undriven.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub power_flags: Vec<Terminal>,
}
/// Current document format version.
pub const DOCUMENT_VERSION: u32 = 4;
/// Format version of wiring diagrams. It is newer than a schematic's so that earlier
/// builds refuse a wiring diagram instead of opening it as a schematic.
pub const WIRING_DOCUMENT_VERSION: u32 = 5;
impl Default for Document {
    fn default() -> Self {
        Self {
            version: DOCUMENT_VERSION,
            kind: DocumentKind::Schematic,
            title: "Untitled schematic".into(),
            components: vec![],
            wires: vec![],
            junctions: BTreeSet::new(),
            labels: vec![],
            board: board::Board::default(),
            no_connects: vec![],
            power_flags: vec![],
        }
    }
}
/// One row of a wiring diagram's connection list.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Connection {
    pub connection_id: String,
    pub from_ref_des: String,
    pub from_port: String,
    pub to_ref_des: String,
    pub to_port: String,
    pub stock_part_number: String,
}
impl Document {
    /// An empty document of the given kind.
    pub fn new(kind: DocumentKind) -> Self {
        match kind {
            DocumentKind::Schematic => Self::default(),
            DocumentKind::Wiring => Self {
                version: WIRING_DOCUMENT_VERSION,
                kind,
                title: "Untitled wiring diagram".into(),
                ..Self::default()
            },
        }
    }
    pub fn place(&mut self, mut symbol: Symbol, at: Point, rotation: u8) -> Uuid {
        // A part saved before gates existed places as gates too.
        symbol.migrate_legacy_units();
        let mut n = 1;
        while self
            .components
            .iter()
            .any(|c| c.reference == format!("{}{n}", symbol.reference_prefix))
        {
            n += 1;
        }
        let id = Uuid::new_v4();
        self.components.push(Component {
            id,
            reference: format!("{}{n}", symbol.reference_prefix),
            value: symbol.default_value(),
            symbol,
            at,
            rotation: rotation % 4,
            part: None,
            pads: None,
            gates: vec![],
        });
        self.components.last_mut().unwrap().sync_gates();
        let flags: Vec<_> = self
            .components
            .last()
            .unwrap()
            .symbol
            .pins
            .iter()
            .filter_map(|p| {
                let symbol = &self.components.last().unwrap().symbol;
                symbol
                    .power_net
                    .clone()
                    .or_else(|| {
                        (p.hidden && p.electrical_type == "power_in" && p.name != "~")
                            .then(|| p.name.clone())
                    })
                    .map(|name| (p.number.clone(), name))
            })
            .collect();
        for (pin, name) in flags {
            let _ = self.set_net_flag(Terminal { component: id, pin }, &name);
        }
        id
    }
    /// A free wire. Wiring diagrams have none; their wires come from [`Document::connect`].
    /// Place a part: its symbol and pads are copied into the document, and the component
    /// remembers which part and which version they came from. `reference` is the label
    /// the host gives it, or the next free one when it has none.
    pub fn place_part(
        &mut self,
        part: PartSource,
        symbol: Symbol,
        pads: Option<board::Footprint>,
        reference: Option<String>,
        at: Point,
        rotation: u8,
    ) -> Result<Uuid, String> {
        if let Some(instance) = &part.instance {
            if self.components.iter().any(|c| c.part.as_ref().and_then(|p| p.instance.as_ref()) == Some(instance)) {
                return Err("This native assembly instance is already placed".into());
            }
        }
        let id = self.place(symbol, at, rotation);
        if let Some(reference) = reference {
            self.set_reference(id, &reference).inspect_err(|_| {
                self.delete_component(id);
            })?;
        }
        let c = self.components.iter_mut().find(|c| c.id == id).unwrap();
        c.part = Some(part);
        c.pads = pads;
        Ok(id)
    }
    /// Give a placed part the label a host keeps for it, such as its reference
    /// designator. Labels are nonempty and unique in the document.
    pub fn set_reference(&mut self, id: Uuid, reference: &str) -> Result<(), String> {
        let reference = reference.trim();
        if reference.is_empty() {
            return Err("A label cannot be empty".into());
        }
        if self
            .components
            .iter()
            .any(|c| c.id != id && c.reference == reference)
        {
            return Err(format!("{reference} is already used by another part"));
        }
        let c = self
            .components
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or("Component no longer exists")?;
        c.reference = reference.into();
        Ok(())
    }
    /// Give each of `ids` the value `value` (`10k`), as one edit: the value the
    /// sheet draws under the reference, and the one the BOM groups by and the
    /// netlist and pick-and-place write. Taken as given, as the Inspector's field
    /// types it (trimming there would eat the space of `10 k` as it is typed); an
    /// empty value is allowed, as KiCad allows it. Returns how many changed.
    pub fn set_value(&mut self, ids: &[Uuid], value: &str) -> usize {
        let mut changed = 0;
        for c in self.components.iter_mut().filter(|c| ids.contains(&c.id)) {
            if c.value != value {
                c.value = value.into();
                changed += 1;
            }
        }
        changed
    }
    /// The components that still carry the value their symbol gave them when
    /// they were placed ([`Symbol::default_value`]) and are placed from the same
    /// symbol as `id`, other than `id` itself, in reference order: the parts a
    /// value typed for `id` is offered to as well.
    pub fn unvalued_siblings(&self, id: Uuid) -> Vec<Uuid> {
        let Some(c) = self.components.iter().find(|c| c.id == id) else {
            return vec![];
        };
        let mut siblings: Vec<&Component> = self
            .components
            .iter()
            .filter(|o| {
                o.id != id
                    && o.is_valued_part()
                    && o.symbol.library_id == c.symbol.library_id
                    && o.value == o.symbol.default_value()
            })
            .collect();
        siblings.sort_by(|a, b| footprint::natural_cmp(&a.reference, &b.reference));
        siblings.into_iter().map(|o| o.id).collect()
    }
    /// Every part the document has components from, with the version each was copied
    /// from. A host compares these against its own parts to find what to refresh.
    pub fn parts(&self) -> Vec<PartSource> {
        let mut parts: Vec<_> = self
            .components
            .iter()
            .filter_map(|c| c.part.clone())
            .collect();
        parts.sort();
        parts.dedup();
        parts
    }
    /// Take a new version of one part into every component placed from it: its symbol
    /// and pads are copied again, and each component records `signature`. `renames`
    /// maps a pin label of the old version to its label in the new one; connections on
    /// a renamed pin move to its new label, connections on the pins whose labels remain
    /// stay, and the rest go. Returns how many components were refreshed. A refusal
    /// leaves the document as it was.
    pub fn refresh_part(
        &mut self,
        key: &str,
        symbol: &Symbol,
        pads: Option<&board::Footprint>,
        signature: &str,
        renames: &BTreeMap<String, String>,
    ) -> Result<usize, String> {
        if signature.trim().is_empty() {
            return Err("A part needs a key and a signature".into());
        }
        let stale: Vec<Uuid> = self
            .components
            .iter()
            .filter(|c| {
                c.part
                    .as_ref()
                    .is_some_and(|p| p.key == key && p.signature != signature)
            })
            .map(|c| c.id)
            .collect();
        let mut draft = self.clone();
        for id in &stale {
            draft.rebind(*id, symbol, pads, renames, |part| part.signature = signature.into())?;
        }
        *self = draft;
        Ok(stale.len())
    }
    /// Take a new version of a part into ONE placed component — the
    /// per-component half of [`Document::refresh_part`], for a host that
    /// decides component by component which version each carries.
    ///
    /// A host whose parts nest cannot name one key and mean them all: the same
    /// key spells a different part at every level of an assembly tree, and one
    /// occurrence's version lives in its OWN parent's library. Such a host
    /// resolves each component's device itself and hands the result here.
    /// `renames` and the atomicity are [`Document::refresh_part`]'s; only the
    /// [`PartSource`]'s `signature` is re-stamped, so a component keeps the key
    /// it was placed from — [`Document::relink`] is the call that moves that.
    pub fn refresh_component(
        &mut self,
        component: Uuid,
        symbol: &Symbol,
        pads: Option<&board::Footprint>,
        signature: &str,
        renames: &BTreeMap<String, String>,
    ) -> Result<(), String> {
        if signature.trim().is_empty() {
            return Err("A part needs a key and a signature".into());
        }
        let mut draft = self.clone();
        draft.rebind(component, symbol, pads, renames, |part| part.signature = signature.into())?;
        *self = draft;
        Ok(())
    }
    /// Copy one part version into ONE placed component: its terminals move by
    /// `renames`, its symbol and pads are replaced, its board placement follows
    /// the new footprint (and goes when the part has none), and `bind` says what
    /// becomes of its [`PartSource`]. The ONE place a component takes a new
    /// version of a part, whether that is the same part refreshed
    /// ([`Document::refresh_part`], [`Document::refresh_component`]) or a
    /// different device ([`Document::relink`]).
    fn rebind(
        &mut self,
        id: Uuid,
        symbol: &Symbol,
        pads: Option<&board::Footprint>,
        renames: &BTreeMap<String, String>,
        bind: impl FnOnce(&mut PartSource),
    ) -> Result<(), String> {
        self.rename_terminals(id, renames);
        self.replace_component_symbol(id, symbol.clone())?;
        let c = self
            .components
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or("Component no longer exists")?;
        c.pads = pads.cloned();
        if let Some(part) = &mut c.part {
            bind(part);
        }
        if let Some(pads) = pads
            && let Some(placement) = self.board.placements.iter_mut().find(|p| p.component == id)
        {
            placement.footprint = pads.clone();
        }
        if pads.is_none() {
            self.board.placements.retain(|placement| placement.component != id);
        }
        Ok(())
    }
    /// Bind a placed component to a DIFFERENT device — the resolving half of
    /// the UNLINKED lane.
    ///
    /// A sheet component names its device by OCCURRENCE ([`PartSource::instance`]),
    /// and neither half of that is stable on its own: an occurrence chain
    /// changes whenever the device is moved in the assembly tree, and the
    /// parts-library key is not stable across a part's version history. When
    /// the binding cannot be resolved the component goes UNLINKED and the USER
    /// picks its device; this is what that pick does. Nothing is guessed at
    /// here, and nothing but the binding moves: the component keeps where it
    /// sits, what it is called, and every wire landing on a pin it still has.
    ///
    /// A device already placed on this sheet is refused, as
    /// [`Document::place_part`] refuses one.
    pub fn relink(
        &mut self,
        component: Uuid,
        part: PartSource,
        symbol: &Symbol,
        pads: Option<&board::Footprint>,
        renames: &BTreeMap<String, String>,
    ) -> Result<(), String> {
        if part.signature.trim().is_empty() {
            return Err("A part needs a key and a signature".into());
        }
        if !self.components.iter().any(|c| c.id == component) {
            return Err("Component no longer exists".into());
        }
        if let Some(instance) = &part.instance
            && self.components.iter().any(|c| {
                c.id != component
                    && c.part.as_ref().and_then(|p| p.instance.as_ref()) == Some(instance)
            })
        {
            return Err("This native assembly instance is already placed".into());
        }
        let mut draft = self.clone();
        // A component drawn without a part has none to bind; give it one, so a
        // legacy symbol can be linked to a device for the first time.
        if let Some(c) = draft.components.iter_mut().find(|c| c.id == component)
            && c.part.is_none()
        {
            c.part = Some(part.clone());
        }
        draft.rebind(component, symbol, pads, renames, |bound| *bound = part)?;
        *self = draft;
        Ok(())
    }
    /// Move every terminal of component `id` from an old pin label to its new one.
    /// All move at once, so two labels that trade places swap rather than merge.
    fn rename_terminals(&mut self, id: Uuid, renames: &BTreeMap<String, String>) {
        let rename = |terminal: &mut Terminal| {
            if terminal.component == id
                && let Some(label) = renames.get(&terminal.pin)
            {
                terminal.pin = label.clone();
            }
        };
        let terminals = self
            .wires
            .iter_mut()
            .flat_map(|w| [w.start.as_mut(), w.end.as_mut()])
            .chain(self.labels.iter_mut().map(|l| l.terminal.as_mut()));
        for terminal in terminals.flatten() {
            rename(terminal);
        }
        // The check's markers follow their pin the same way.
        for terminal in self.no_connects.iter_mut().chain(self.power_flags.iter_mut()) {
            rename(terminal);
        }
    }
    pub fn add_wire(&mut self, a: Point, b: Point) {
        if self.kind == DocumentKind::Schematic
            && a != b
            && (a.x == b.x || a.y == b.y)
            && !self
                .wires
                .iter()
                .any(|w| (w.a == a && w.b == b) || (w.a == b && w.b == a))
        {
            let mut wire = Wire::new(a, b);
            wire.start = self.terminal_at(a);
            wire.end = self.terminal_at(b);
            self.wires.push(wire);
        }
    }
    /// Create or rename the flag attached to this pin. Equal names share a net.
    pub fn set_net_flag(&mut self, terminal: Terminal, name: &str) -> Result<Uuid, String> {
        if self.kind == DocumentKind::Wiring {
            return Err("A wiring diagram has no nets".into());
        }
        let name = name.trim();
        if name.is_empty() {
            return Err("Enter a net name".into());
        }
        let at = self
            .terminal_position(&terminal)
            .ok_or("Terminal no longer exists")?;
        if let Some(label) = self
            .labels
            .iter_mut()
            .find(|l| l.terminal.as_ref() == Some(&terminal))
        {
            label.name = name.into();
            label.at = at;
            return Ok(label.id);
        }
        let id = Uuid::new_v4();
        self.labels.push(Label {
            flag: true,
            rotation: 0,
            id,
            name: name.into(),
            at,
            terminal: Some(terminal),
        });
        Ok(id)
    }
    /// Move the tip; free wire endpoints follow, and direct pin contact is
    /// recomputed at the new location.
    pub fn transform_label(&mut self, id: Uuid, at: Point, rotation: u8) {
        let terminal = self.terminal_at(at);
        let Some(label) = self.labels.iter_mut().find(|l| l.id == id) else {
            return;
        };
        let old = label.at;
        label.flag |= label.terminal.is_some();
        label.at = at;
        label.rotation = rotation % 4;
        label.terminal = terminal;
        for wire in &mut self.wires {
            let a = if wire.a == old && wire.start.is_none() {
                at
            } else {
                wire.a
            };
            let b = if wire.b == old && wire.end.is_none() {
                at
            } else {
                wire.b
            };
            wire.follow(a, b);
        }
    }
    pub fn terminal_position(&self, terminal: &Terminal) -> Option<Point> {
        let c = self
            .components
            .iter()
            .find(|c| c.id == terminal.component)?;
        c.symbol
            .pins
            .iter()
            .find(|p| p.number == terminal.pin)
            .map(|p| c.pin_at(p))
    }
    pub fn terminal_at(&self, at: Point) -> Option<Terminal> {
        self.components.iter().find_map(|c| {
            c.symbol
                .pins
                .iter()
                .find(|p| c.pin_at(p) == at)
                .map(|p| Terminal {
                    component: c.id,
                    pin: p.number.clone(),
                })
        })
    }
    /// Short orthogonal route with outward terminal leads. Avoid crossing other
    /// terminal positions; this is deliberately not a full obstacle router.
    pub fn connection_path(&self, start: &Terminal, end: &Terminal) -> Option<Vec<Point>> {
        let a = self.terminal_position(start)?;
        let b = self.terminal_position(end)?;
        let outward = |t: &Terminal, at: Point| -> Option<Point> {
            let c = self.components.iter().find(|c| c.id == t.component)?;
            let p = c.symbol.pins.iter().find(|p| p.number == t.pin)?;
            let inside = c.pin_end(p);
            Some(Point::new(
                (at.x - inside.x).signum() * 2540,
                (at.y - inside.y).signum() * 2540,
            ))
        };
        let da = outward(start, a)?;
        let db = outward(end, b)?;
        let sa = a.offset(da);
        let sb = b.offset(db);
        let pins: Vec<_> = self
            .components
            .iter()
            .flat_map(|c| c.symbol.pins.iter().map(|p| c.pin_at(p)))
            .collect();
        let min_x = pins
            .iter()
            .map(|p| p.x)
            .min()
            .unwrap_or(0)
            .min(sa.x)
            .min(sb.x)
            - 5080;
        let max_x = pins
            .iter()
            .map(|p| p.x)
            .max()
            .unwrap_or(0)
            .max(sa.x)
            .max(sb.x)
            + 5080;
        let min_y = pins
            .iter()
            .map(|p| p.y)
            .min()
            .unwrap_or(0)
            .min(sa.y)
            .min(sb.y)
            - 5080;
        let max_y = pins
            .iter()
            .map(|p| p.y)
            .max()
            .unwrap_or(0)
            .max(sa.y)
            .max(sb.y)
            + 5080;
        let mut candidates = vec![
            vec![a, sa, Point::new(sb.x, sa.y), sb, b],
            vec![a, sa, Point::new(sa.x, sb.y), sb, b],
        ];
        for x in [min_x, max_x] {
            candidates.push(vec![a, sa, Point::new(x, sa.y), Point::new(x, sb.y), sb, b]);
        }
        for y in [min_y, max_y] {
            candidates.push(vec![a, sa, Point::new(sa.x, y), Point::new(sb.x, y), sb, b]);
        }
        candidates
            .into_iter()
            .filter_map(|p| {
                let mut w = Wire::new(a, b);
                w.set_points(p);
                let p = w.points();
                let first = Point::new(p[1].x - a.x, p[1].y - a.y);
                let n = p.len();
                let last = Point::new(p[n - 2].x - b.x, p[n - 2].y - b.y);
                let dot = |x: Point, y: Point| {
                    i64::from(x.x) * i64::from(y.x) + i64::from(x.y) * i64::from(y.y)
                };
                if dot(first, da) <= 0 || dot(last, db) <= 0 {
                    return None;
                }
                if pins
                    .iter()
                    .filter(|p| **p != a && **p != b)
                    .any(|pin| p.windows(2).any(|s| on_segment(*pin, s[0], s[1])))
                {
                    return None;
                }
                let length: u64 = p
                    .windows(2)
                    .map(|s| {
                        u64::from(s[0].x.abs_diff(s[1].x)) + u64::from(s[0].y.abs_diff(s[1].y))
                    })
                    .sum();
                Some((length, p))
            })
            .min_by_key(|(length, _)| *length)
            .map(|(_, p)| p)
    }
    pub fn target_position(&self, target: &ConnectionTarget) -> Option<Point> {
        match target {
            ConnectionTarget::Terminal(t) => self.terminal_position(t),
            ConnectionTarget::Junction(p) => {
                (self.junctions.contains(p) || self.labels.iter().any(|l| l.at == *p)).then_some(*p)
            }
        }
    }
    pub fn target_path(
        &self,
        start: &ConnectionTarget,
        end: &ConnectionTarget,
    ) -> Option<Vec<Point>> {
        let a = self.target_position(start)?;
        let b = self.target_position(end)?;
        if let (ConnectionTarget::Terminal(start), ConnectionTarget::Terminal(end)) = (start, end) {
            return Some(
                self.connection_path(start, end)
                    .unwrap_or_else(|| vec![a, Point::new(b.x, a.y), b]),
            );
        }
        Some(vec![a, Point::new(b.x, a.y), b])
    }
    pub fn connect_targets(
        &mut self,
        start: ConnectionTarget,
        end: ConnectionTarget,
    ) -> Option<Uuid> {
        if let (ConnectionTarget::Terminal(a), ConnectionTarget::Terminal(b)) = (&start, &end) {
            return self.connect(a.clone(), b.clone());
        }
        if self.kind == DocumentKind::Wiring {
            return None; // Connections run pin to pin; there are no junctions.
        }
        let a = self.target_position(&start)?;
        let b = self.target_position(&end)?;
        if a == b
            || self
                .wires
                .iter()
                .any(|w| (w.a == a && w.b == b) || (w.a == b && w.b == a))
        {
            return None;
        }
        let mut w = Wire::new(a, b);
        w.set_points(self.target_path(&start, &end)?);
        if let ConnectionTarget::Terminal(t) = start {
            w.start = Some(t);
        }
        if let ConnectionTarget::Terminal(t) = end {
            w.end = Some(t);
        }
        let id = w.id;
        self.wires.push(w);
        Some(id)
    }
    pub fn connect(&mut self, start: Terminal, end: Terminal) -> Option<Uuid> {
        if start == end {
            return None;
        }
        let a = self.terminal_position(&start)?;
        let b = self.terminal_position(&end)?;
        if a == b {
            return None;
        }
        if self.wires.iter().any(|w| {
            (w.start.as_ref() == Some(&start) && w.end.as_ref() == Some(&end))
                || (w.start.as_ref() == Some(&end) && w.end.as_ref() == Some(&start))
        }) {
            return None;
        }
        let wiring = self.kind == DocumentKind::Wiring;
        if wiring && (self.attached_wire(&start).is_some() || self.attached_wire(&end).is_some()) {
            return None;
        }
        let mut wire = Wire::new(a, b);
        wire.set_points(
            self.connection_path(&start, &end)
                .unwrap_or_else(|| vec![a, Point::new(b.x, a.y), b]),
        );
        wire.start = Some(start);
        wire.end = Some(end);
        if wiring {
            let mut n = 1;
            while self
                .wires
                .iter()
                .any(|w| w.connection_id == format!("W{n}"))
            {
                n += 1;
            }
            wire.connection_id = format!("W{n}");
        }
        let id = wire.id;
        self.wires.push(wire);
        Some(id)
    }
    /// The wire with an end attached to this pin, if any.
    pub fn attached_wire(&self, terminal: &Terminal) -> Option<&Wire> {
        self.wires
            .iter()
            .find(|w| w.start.as_ref() == Some(terminal) || w.end.as_ref() == Some(terminal))
    }
    /// A pin as reference and pin number, such as `R1.2`.
    pub fn terminal_name(&self, terminal: &Terminal) -> String {
        let reference = self
            .components
            .iter()
            .find(|c| c.id == terminal.component)
            .map_or("?", |c| c.reference.as_str());
        format!("{reference}.{}", terminal.pin)
    }
    /// A wiring diagram's point-to-point connections, ordered by connection ID. Each runs
    /// from the pin its wire was drawn from; a port is a pin number.
    pub fn connections(&self) -> Vec<Connection> {
        let reference = |t: &Terminal| {
            self.components
                .iter()
                .find(|c| c.id == t.component)
                .map(|c| c.reference.clone())
        };
        let mut rows: Vec<_> = self
            .wires
            .iter()
            .filter_map(|w| {
                let (start, end) = (w.start.as_ref()?, w.end.as_ref()?);
                Some(Connection {
                    connection_id: w.connection_id.clone(),
                    from_ref_des: reference(start)?,
                    from_port: start.pin.clone(),
                    to_ref_des: reference(end)?,
                    to_port: end.pin.clone(),
                    stock_part_number: w.stock_part_number.clone(),
                })
            })
            .collect();
        rows.sort_by(|a, b| id_order(&a.connection_id).cmp(&id_order(&b.connection_id)));
        rows
    }
    /// The connection list as CSV: a header row, then one row per connection.
    pub fn connections_csv(&self) -> Result<String, String> {
        if self.kind != DocumentKind::Wiring {
            return Err("Only a wiring diagram has a connection list".into());
        }
        self.validate()?;
        let mut csv =
            String::from("connectionID,fromRefDes,fromPort,toRefDes,toPort,stockPartNumber\n");
        for c in self.connections() {
            let fields = [
                &c.connection_id,
                &c.from_ref_des,
                &c.from_port,
                &c.to_ref_des,
                &c.to_port,
                &c.stock_part_number,
            ];
            csv.push_str(&fields.map(|f| csv_field(f)).join(","));
            csv.push('\n');
        }
        Ok(csv)
    }
    /// Bring every component whose symbol the KiCad import wrote before gates
    /// existed up to the gate shape ([`Symbol::migrate_legacy_units`]), each
    /// gate placed exactly where the symbol drawn whole put it, so no pin, wire
    /// or flag moves. Returns the references of the components it changed.
    ///
    /// A host calls this where it OPENS a stored sheet, and adopts the result
    /// as part of the document rather than as an edit: nothing the user drew
    /// changes, and a sheet not saved again is migrated the same way the next
    /// time it is opened.
    pub fn migrate_legacy_units(&mut self) -> Vec<String> {
        let mut migrated = vec![];
        for c in &mut self.components {
            if c.symbol.migrate_legacy_units() {
                c.gates.clear();
                c.sync_gates();
                migrated.push(c.reference.clone());
            }
        }
        migrated
    }
    /// Replace one instance's definition, retaining connectivity by pin number.
    pub fn replace_component_symbol(&mut self, id: Uuid, mut symbol: Symbol) -> Result<(), String> {
        symbol.migrate_legacy_units();
        let mut draft = self.clone();
        let c = draft
            .components
            .iter()
            .find(|c| c.id == id)
            .ok_or("Component no longer exists")?;
        let (at, rotation) = (c.at, c.rotation);
        draft.transform_component(id, at, rotation); // bind legacy endpoints before changing geometry
        let valid_terminal =
            |t: &Terminal| t.component != id || symbol.pins.iter().any(|p| p.number == t.pin);
        if draft.kind == DocumentKind::Wiring {
            // A connection needs both of its pins; one that loses a pin goes with it.
            draft.wires.retain(|w| {
                w.start.as_ref().is_none_or(valid_terminal)
                    && w.end.as_ref().is_none_or(valid_terminal)
            });
        }
        for wire in &mut draft.wires {
            if wire.start.as_ref().is_some_and(|t| !valid_terminal(t)) {
                wire.start = None;
            }
            if wire.end.as_ref().is_some_and(|t| !valid_terminal(t)) {
                wire.end = None;
            }
        }
        for label in &mut draft.labels {
            if label.terminal.as_ref().is_some_and(|t| !valid_terminal(t)) {
                label.flag = true;
                label.terminal = None;
            }
        }
        draft.no_connects.retain(valid_terminal);
        draft.power_flags.retain(valid_terminal);
        let c = draft.components.iter_mut().find(|c| c.id == id).unwrap();
        c.symbol = symbol;
        c.sync_gates();
        draft.transform_component(id, at, rotation);
        draft.validate()?;
        *self = draft;
        Ok(())
    }
    /// Infer attachments on legacy/free wires before moving, then keep them attached.
    /// A component whose gates are placed apart moves as one: every gate goes
    /// with it, turned about `at` by the same quarter turns.
    pub fn transform_component(&mut self, id: Uuid, at: Point, rotation: u8) {
        self.reposition(id, |c| {
            let turn = (rotation % 4 + 4 - c.rotation) % 4;
            for g in &mut c.gates {
                g.at = Point::new(g.at.x - c.at.x, g.at.y - c.at.y).rotate(turn).offset(at);
                g.rotation = (g.rotation + turn) % 4;
            }
            let rotated = c.rotation != rotation % 4;
            c.at = at;
            c.rotation = rotation % 4;
            rotated
        });
    }
    /// Move and turn ONE gate of a component whose gates are placed apart
    /// ([`Component::gates`]); its wires and flags follow as they follow a
    /// component. Nothing happens for a gate the component does not place.
    pub fn transform_gate(&mut self, id: Uuid, gate: u32, at: Point, rotation: u8) {
        self.reposition(id, |c| {
            let Some(g) = c.gates.iter_mut().find(|g| g.gate == gate) else {
                return false;
            };
            let rotated = g.rotation != rotation % 4;
            g.at = at;
            g.rotation = rotation % 4;
            rotated
        });
    }
    /// Change where component `id` is drawn with `change`, which says whether it
    /// turned anything; wires attached to it follow (a turn reroutes them) and
    /// its flags move with their pins.
    fn reposition(&mut self, id: Uuid, change: impl FnOnce(&mut Component) -> bool) {
        let bindings: Vec<_> = self
            .wires
            .iter()
            .map(|w| {
                (
                    w.start.clone().or_else(|| self.terminal_at(w.a)),
                    w.end.clone().or_else(|| self.terminal_at(w.b)),
                )
            })
            .collect();
        // Which way each flag's pin on this component points, before the turn.
        let leads_before: Vec<_> = self
            .labels
            .iter()
            .map(|l| l.terminal.as_ref().filter(|t| t.component == id).and_then(|t| self.lead(t)))
            .collect();
        let Some(c) = self.components.iter_mut().find(|c| c.id == id) else {
            return;
        };
        let rotated = change(c);
        let routes: Vec<_> = bindings
            .iter()
            .map(|(a, b)| {
                if rotated && a.as_ref().is_some_and(|t| t.component == id)
                    || rotated && b.as_ref().is_some_and(|t| t.component == id)
                {
                    a.as_ref()
                        .zip(b.as_ref())
                        .and_then(|(a, b)| self.connection_path(a, b))
                } else {
                    None
                }
            })
            .collect();
        let positions: Vec<_> = bindings
            .iter()
            .map(|(a, b)| {
                (
                    a.as_ref().and_then(|t| self.terminal_position(t)),
                    b.as_ref().and_then(|t| self.terminal_position(t)),
                )
            })
            .collect();
        for (((w, (start, end)), (a, b)), route) in self
            .wires
            .iter_mut()
            .zip(bindings)
            .zip(positions)
            .zip(routes)
        {
            w.start = start;
            w.end = end;
            w.follow(a.unwrap_or(w.a), b.unwrap_or(w.b));
            if let Some(route) = route {
                w.set_points(route);
            }
        }
        let flag_positions: Vec<_> = self
            .labels
            .iter()
            .map(|l| l.terminal.as_ref().and_then(|t| self.terminal_position(t)))
            .collect();
        let leads_after: Vec<_> = self
            .labels
            .iter()
            .map(|l| l.terminal.as_ref().filter(|t| t.component == id).and_then(|t| self.lead(t)))
            .collect();
        for (((label, at), before), after) in self
            .labels
            .iter_mut()
            .zip(flag_positions)
            .zip(leads_before)
            .zip(leads_after)
        {
            if let Some(at) = at {
                label.at = at;
            }
            // A flag turns with the pin it stands on, by the same quarter
            // turns, so one set to point away from a gate still does after the
            // gate turns. Relative, so a flag the user turned keeps that.
            if let (Some(before), Some(after)) = (before, after)
                && let Some(turn) = (0..4u8).find(|&k| before.rotate(k) == after)
            {
                label.rotation = (label.rotation + turn) % 4;
            }
        }
    }
    /// The way a pin's lead runs on the sheet, from its terminal toward its
    /// body, as a unit step along one axis; `None` for a pin of no length or
    /// a terminal that no longer exists.
    fn lead(&self, terminal: &Terminal) -> Option<Point> {
        let c = self.components.iter().find(|c| c.id == terminal.component)?;
        let pin = c.symbol.pins.iter().find(|p| p.number == terminal.pin)?;
        let (at, end) = (c.pin_at(pin), c.pin_end(pin));
        let step = Point::new((end.x - at.x).signum(), (end.y - at.y).signum());
        (step != Point::default()).then_some(step)
    }
    /// Where a new component drawn with `symbol` can go without covering one already
    /// on the sheet: the first free cell, row by row and eight to a row, of a grid
    /// that starts at the sheet origin. A cell is the symbol's own box plus a gap:
    /// 10.16 mm across, which is room for the reference and value an editor writes
    /// to the right of a symbol (two lines of text, some 80 points wide at the
    /// default zoom), and 5.08 mm down. The origin itself when it is free, so the
    /// first part lands where it always did.
    ///
    /// A host that seats parts on its own, with no pointer to say where, asks this
    /// rather than stacking every part at the origin: stacked parts publish the same
    /// rect, and a click or a drag reaches only the one drawn last (the eCAD workflow
    /// audit, issue 6).
    pub fn open_spot(&self, symbol: &Symbol) -> Point {
        const ACROSS: i32 = 10160;
        const DOWN: i32 = 5080;
        let (lo, hi) = symbol_extent(symbol, 0);
        let cell = |span: i32, gap: i32| (span.max(0) + gap + 1269) / 1270 * 1270;
        let (w, h) = (cell(hi.x - lo.x, ACROSS), cell(hi.y - lo.y, DOWN));
        let taken: Vec<(Point, Point)> = self
            .components
            .iter()
            .flat_map(Component::sheet_boxes)
            .map(|(_, a, b)| (a, b))
            .collect();
        let (x, y) = (ACROSS / 2, DOWN / 2);
        for row in 0..256 {
            for col in 0..8 {
                let at = Point::new(col * w, row * h);
                let (a, b) = (lo.offset(at), hi.offset(at));
                let clear = taken.iter().all(|(c, d)| {
                    a.x - x > d.x || b.x + x < c.x || a.y - y > d.y || b.y + y < c.y
                });
                if clear {
                    return at;
                }
            }
        }
        Point::default()
    }
    /// Whether component `id` covers part of another component, each taken as the box
    /// its symbol spans grown by half a sheet grid step (635 µm) on every side — so a
    /// symbol that is only a row of pins, whose box has no height, still covers one
    /// drawn on top of it, and parts a whole grid step apart do not.
    pub fn covers_another(&self, id: Uuid) -> bool {
        let spans = |c: &Component| -> Vec<(Point, Point)> {
            c.sheet_boxes()
                .into_iter()
                .map(|(_, a, b)| (a.offset(Point::new(-635, -635)), b.offset(Point::new(635, 635))))
                .collect()
        };
        let Some(mine) = self.components.iter().find(|c| c.id == id).map(spans) else {
            return false;
        };
        self.components
            .iter()
            .filter(|c| c.id != id)
            .flat_map(spans)
            .any(|(c, d)| mine.iter().any(|(a, b)| a.x < d.x && c.x < b.x && a.y < d.y && c.y < b.y))
    }
    pub fn delete_component(&mut self, id: Uuid) {
        self.components.retain(|c| c.id != id);
        self.board.placements.retain(|p| p.component != id);
        self.labels
            .retain(|l| !l.terminal.as_ref().is_some_and(|t| t.component == id));
        self.no_connects.retain(|t| t.component != id);
        self.power_flags.retain(|t| t.component != id);
        if self.kind == DocumentKind::Wiring {
            // A connection needs both of its pins.
            let attached = |t: &Option<Terminal>| t.as_ref().is_some_and(|t| t.component == id);
            self.wires
                .retain(|w| !attached(&w.start) && !attached(&w.end));
        }
        for w in &mut self.wires {
            if w.start.as_ref().is_some_and(|t| t.component == id) {
                w.start = None;
            }
            if w.end.as_ref().is_some_and(|t| t.component == id) {
                w.end = None;
            }
        }
    }
    pub fn from_json(text: &str) -> Result<Self, String> {
        Self::migrated(serde_json::from_str(text).map_err(|e| e.to_string())?)
    }
    pub fn to_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string_pretty(self).map_err(|e| e.to_string())
    }
    /// Read a document kept inside a host's own JSON file, with the same migration and
    /// checks as [`Document::from_json`].
    pub fn from_value(value: serde_json::Value) -> Result<Self, String> {
        Self::migrated(serde_json::from_value(value).map_err(|e| e.to_string())?)
    }
    /// The document as a JSON value for a host's own file, checked as
    /// [`Document::to_json`] checks it.
    pub fn to_value(&self) -> Result<serde_json::Value, String> {
        self.validate()?;
        serde_json::to_value(self).map_err(|e| e.to_string())
    }
    fn migrated(mut doc: Self) -> Result<Self, String> {
        if (1..DOCUMENT_VERSION).contains(&doc.version) {
            doc.version = DOCUMENT_VERSION;
        }
        doc.validate()?;
        Ok(doc)
    }
    pub fn validate(&self) -> Result<(), String> {
        let version = match self.kind {
            DocumentKind::Schematic => DOCUMENT_VERSION,
            DocumentKind::Wiring => WIRING_DOCUMENT_VERSION,
        };
        if self.version != version {
            return Err(format!("Unsupported document version {}", self.version));
        }
        let mut ids = BTreeSet::new();
        let mut refs = BTreeSet::new();
        for id in self
            .components
            .iter()
            .map(|c| c.id)
            .chain(self.wires.iter().map(|w| w.id))
            .chain(self.labels.iter().map(|l| l.id))
            .chain(self.board.tracks.iter().map(|t| t.id))
            .chain(self.board.vias.iter().map(|v| v.id))
        {
            if !ids.insert(id) {
                return Err("Duplicate object ID".into());
            }
        }
        let valid = |p: Point| p.x.abs_diff(0) <= 100_000_000 && p.y.abs_diff(0) <= 100_000_000;
        for c in &self.components {
            if c.reference.trim().is_empty() || !refs.insert(&c.reference) {
                return Err("Component references must be nonempty and unique".into());
            }
            if !valid(c.at) || c.rotation > 3 {
                return Err("Invalid component placement".into());
            }
            let mut gates = BTreeSet::new();
            for g in &c.gates {
                if g.gate == 0 || !gates.insert(g.gate) || !valid(g.at) || !valid(g.pivot) || g.rotation > 3 {
                    return Err("Invalid gate placement".into());
                }
            }
            let mut pins = BTreeSet::new();
            for pin in &c.symbol.pins {
                if pin.number.is_empty()
                    || !pins.insert(&pin.number)
                    || !valid(pin.at)
                    || !valid(pin.end)
                {
                    return Err("Invalid or duplicate symbol pin".into());
                }
            }
            if let Some(part) = &c.part
                && (part.key.trim().is_empty() || part.signature.trim().is_empty())
            {
                return Err("A part needs a key and a signature".into());
            }
            if let Some(pads) = &c.pads {
                pads.validate_placeable()?;
            }
            for g in &c.symbol.graphics {
                match g {
                    Graphic::Path(p) if p.iter().any(|p| !valid(*p)) => {
                        return Err("Invalid graphic coordinates".into());
                    }
                    Graphic::Circle { center, radius }
                        if !valid(*center) || !(0..=100_000_000).contains(radius) =>
                    {
                        return Err("Invalid circle".into());
                    }
                    Graphic::Text { at, size, .. }
                        if !valid(*at) || !(1..=100_000_000).contains(size) =>
                    {
                        return Err("Invalid symbol text".into());
                    }
                    _ => {}
                }
            }
        }
        for w in &self.wires {
            let points = w.points();
            if points.iter().any(|p| !valid(*p))
                || points
                    .windows(2)
                    .any(|p| p[0] == p[1] || (p[0].x != p[1].x && p[0].y != p[1].y))
            {
                return Err("Wire paths must contain nonzero orthogonal segments".into());
            }
            for (t, p) in [(&w.start, w.a), (&w.end, w.b)] {
                if let Some(t) = t
                    && self.terminal_position(t) != Some(p)
                {
                    return Err("Wire terminal attachment is missing or out of date".into());
                }
            }
        }
        for label in &self.labels {
            if let Some(t) = &label.terminal
                && self.terminal_position(t) != Some(label.at)
            {
                return Err("Net flag terminal attachment is missing or out of date".into());
            }
        }
        if self.junctions.iter().any(|p| !valid(*p))
            || self
                .labels
                .iter()
                .any(|l| l.name.trim().is_empty() || l.rotation > 3 || !valid(l.at))
        {
            return Err("Invalid junction or label".into());
        }
        if self.kind == DocumentKind::Wiring {
            self.validate_wiring()?;
        }
        self.board
            .validate(&self.components.iter().map(|c| c.id).collect())?;
        Ok(())
    }
    /// A wiring diagram has no nets: every wire is a connection between two symbol pins,
    /// no pin takes more than one, and connection IDs are nonempty and unique.
    fn validate_wiring(&self) -> Result<(), String> {
        if !self.junctions.is_empty() || !self.labels.is_empty() {
            return Err("A wiring diagram has no junctions or net labels".into());
        }
        let mut ids = BTreeSet::new();
        let mut pins = BTreeSet::new();
        for w in &self.wires {
            if w.connection_id.trim().is_empty() || !ids.insert(&w.connection_id) {
                return Err("Connection IDs must be nonempty and unique".into());
            }
            let (Some(start), Some(end)) = (&w.start, &w.end) else {
                return Err(format!("Connection {} must join two pins", w.connection_id));
            };
            for t in [start, end] {
                if !pins.insert((t.component, &t.pin)) {
                    return Err(format!(
                        "{} has more than one connection; a wiring diagram pin takes one",
                        self.terminal_name(t)
                    ));
                }
            }
        }
        Ok(())
    }
    /// Rebuild electrical equivalence classes. Interior crossings do not connect
    /// unless a junction, pin, or label is explicitly present there.
    pub fn netlist(&self) -> Netlist {
        let mut points = BTreeSet::new();
        for w in &self.wires {
            points.extend([w.a, w.b]);
        }
        points.extend(self.junctions.iter().copied());
        points.extend(self.labels.iter().map(|l| l.at));
        for c in &self.components {
            points.extend(c.symbol.pins.iter().map(|p| c.pin_at(p)));
        }
        let points: Vec<_> = points.into_iter().collect();
        let index: BTreeMap<_, _> = points.iter().enumerate().map(|(i, p)| (*p, i)).collect();
        let mut groups: Vec<usize> = (0..points.len()).collect();
        fn root(g: &[usize], mut i: usize) -> usize {
            while g[i] != i {
                i = g[i];
            }
            i
        }
        fn join(g: &mut [usize], a: usize, b: usize) {
            let a = root(g, a);
            let b = root(g, b);
            g[b] = a;
        }
        for w in &self.wires {
            let a = index[&w.a];
            let path = w.points();
            for (i, p) in points.iter().enumerate() {
                if path.windows(2).any(|s| on_segment(*p, s[0], s[1])) {
                    join(&mut groups, a, i);
                }
            }
        }
        let mut names = BTreeMap::new();
        for l in &self.labels {
            let i = index[&l.at];
            if let Some(&other) = names.get(&l.name) {
                join(&mut groups, i, other);
            } else {
                names.insert(l.name.clone(), i);
            }
        }
        let mut nets: BTreeMap<usize, Net> = BTreeMap::new();
        for (i, p) in points.iter().enumerate() {
            nets.entry(root(&groups, i)).or_default().points.push(*p);
        }
        for l in &self.labels {
            nets.get_mut(&root(&groups, index[&l.at]))
                .unwrap()
                .labels
                .insert(l.name.clone());
        }
        for c in &self.components {
            for p in &c.symbol.pins {
                nets.get_mut(&root(&groups, index[&c.pin_at(p)]))
                    .unwrap()
                    .pins
                    .push(NetPin {
                        component_id: c.id,
                        reference: c.reference.clone(),
                        number: p.number.clone(),
                        electrical_type: p.electrical_type.clone(),
                    });
            }
        }
        let mut nets: Vec<_> = nets.into_values().collect();
        nets.sort_by_key(|n| n.points[0]);
        let mut diagnostics = vec![];
        for (i, n) in nets.iter_mut().enumerate() {
            n.name = n
                .labels
                .iter()
                .next()
                .cloned()
                .unwrap_or_else(|| format!("N${}", i + 1));
            if n.labels.len() > 1 {
                diagnostics.push(format!(
                    "{}: conflicting labels {}",
                    n.name,
                    n.labels.iter().cloned().collect::<Vec<_>>().join(", ")
                ));
            }
            if n.pins.len() == 1 {
                diagnostics.push(format!(
                    "{}.{} has no other connected pin",
                    n.pins[0].reference, n.pins[0].number
                ));
            }
            if n.pins
                .iter()
                .filter(|p| p.electrical_type == "output")
                .count()
                > 1
            {
                diagnostics.push(format!("{}: multiple output pins", n.name));
            }
        }
        Netlist {
            version: 1,
            nets,
            diagnostics,
        }
    }
}
/// Sorts IDs such as `W2` before `W10`.
fn id_order(id: &str) -> (&str, u64, &str) {
    let prefix = id.trim_end_matches(|c: char| c.is_ascii_digit());
    (prefix, id[prefix.len()..].parse().unwrap_or(0), id)
}
/// The box a symbol covers turned by `rotation` quarter turns about its origin: its
/// pins end to end, its paths, circles and text anchors. A symbol with nothing in
/// it covers its origin.
fn symbol_extent(symbol: &Symbol, rotation: u8) -> (Point, Point) {
    let mut points = vec![Point::default()];
    for pin in &symbol.pins {
        points.extend([pin.at, pin.end]);
    }
    for g in &symbol.graphics {
        match g {
            Graphic::Path(path) => points.extend(path.iter().copied()),
            Graphic::Circle { center, radius } => points.extend([
                Point::new(center.x - radius, center.y - radius),
                Point::new(center.x + radius, center.y + radius),
            ]),
            Graphic::Text { at, .. } => points.push(*at),
        }
    }
    let turned: Vec<Point> = points.into_iter().map(|p| p.rotate(rotation)).collect();
    let lo = Point::new(
        turned.iter().map(|p| p.x).min().unwrap_or(0),
        turned.iter().map(|p| p.y).min().unwrap_or(0),
    );
    let hi = Point::new(
        turned.iter().map(|p| p.x).max().unwrap_or(0),
        turned.iter().map(|p| p.y).max().unwrap_or(0),
    );
    (lo, hi)
}
fn csv_field(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.into()
    }
}
pub fn on_segment(p: Point, a: Point, b: Point) -> bool {
    (a.x == b.x && p.x == a.x && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y))
        || (a.y == b.y && p.y == a.y && p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x))
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Net {
    pub name: String,
    pub labels: BTreeSet<String>,
    pub pins: Vec<NetPin>,
    pub points: Vec<Point>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NetPin {
    pub component_id: Uuid,
    pub reference: String,
    pub number: String,
    pub electrical_type: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Netlist {
    pub version: u32,
    pub nets: Vec<Net>,
    pub diagnostics: Vec<String>,
}

/// Snapshot history: one transaction per gesture; no filesystem or UI dependency.
/// Defaults to a document, and also serves the symbol and footprint editors.
pub struct History<T = Document> {
    undo: Vec<T>,
    redo: Vec<T>,
}
impl<T> Default for History<T> {
    fn default() -> Self {
        Self { undo: Vec::new(), redo: Vec::new() }
    }
}
impl<T: PartialEq> History<T> {
    pub fn record(&mut self, before: T, after: &T) {
        if &before != after {
            self.undo.push(before);
            if self.undo.len() > 100 {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
    }
    pub fn undo(&mut self, doc: &mut T) {
        if let Some(previous) = self.undo.pop() {
            self.redo.push(std::mem::replace(doc, previous));
        }
    }
    pub fn redo(&mut self, doc: &mut T) {
        if let Some(next) = self.redo.pop() {
            self.undo.push(std::mem::replace(doc, next));
        }
    }
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }
}
