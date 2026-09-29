//! Electrical rule check: what a schematic's pins say about how they may be joined,
//! checked against the nets [`Document::netlist`] builds.
//!
//! A reduced form of KiCad's ERC. The pin-to-pin rules are KiCad's default pin map
//! (`erc_settings.cpp`, the twelve electrical types), applied once per net and pair of
//! types rather than once per pair of pins, so a net with three outputs is one finding
//! and not three. On top of the map: a pin joined to nothing, a power input no power
//! output or power flag drives, a no-connect pin that is joined, a no-connect marker on
//! a joined pin, a gate (KiCad's unit) of a part split into gates with nothing joined
//! or not placed at all, two parts with one reference, a net with two names, and a
//! wire end that touches nothing.
//!
//! Every finding carries a place on the sheet to zoom to, and where one edit answers
//! it, that edit ([`ErcFix`]).
use crate::{Document, DocumentKind, Point, Terminal};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// How bad a finding is, as KiCad grades them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ErcSeverity {
    Error,
    Warning,
}
/// What a finding is about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ErcKind {
    /// A pin with no other pin on its net.
    PinNotConnected,
    /// Two pin types the pin map does not allow on one net.
    PinConflict,
    /// A power input pin on a net that no power output pin and no power flag drives.
    PowerNotDriven,
    /// A pin of type no_connect joined to something.
    NoConnectJoined,
    /// A no-connect marker on a pin that is joined after all.
    NoConnectMarkerJoined,
    /// Every pin of one gate of a part split into gates ([`crate::Pin::gate`],
    /// KiCad's unit) joined to nothing.
    UnitUnused,
    /// A gate of a part whose gates are placed apart that has no placement:
    /// KiCad's "missing unit". Its pins are on no sheet, so no other rule reads them.
    UnitMissing,
    /// Two components with one reference.
    DuplicateReference,
    /// Two net names on one net.
    ConflictingLabels,
    /// A wire end that touches no pin, wire, junction or label.
    DanglingWire,
}
impl ErcKind {
    /// The kind's name as `ecad_state` and the scripts read it.
    pub fn key(self) -> &'static str {
        match self {
            Self::PinNotConnected => "pin_not_connected",
            Self::PinConflict => "pin_conflict",
            Self::PowerNotDriven => "power_not_driven",
            Self::NoConnectJoined => "no_connect_joined",
            Self::NoConnectMarkerJoined => "no_connect_marker_joined",
            Self::UnitUnused => "unit_unused",
            Self::UnitMissing => "unit_missing",
            Self::DuplicateReference => "duplicate_reference",
            Self::ConflictingLabels => "conflicting_labels",
            Self::DanglingWire => "dangling_wire",
        }
    }
}
/// The one edit that answers a finding.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ErcFix {
    /// Mark these pins as meant to be left open.
    MarkNoConnect(Vec<Terminal>),
    /// Take the no-connect marker off this pin.
    RemoveNoConnect(Terminal),
    /// Put a power flag on this pin: the net is driven from off the sheet.
    AddPowerFlag(Terminal),
}
impl ErcFix {
    /// The button's words.
    pub fn label(&self) -> &'static str {
        match self {
            Self::MarkNoConnect(pins) if pins.len() > 1 => "Mark its pins no-connect",
            Self::MarkNoConnect(_) => "Mark no-connect",
            Self::RemoveNoConnect(_) => "Remove the no-connect",
            Self::AddPowerFlag(_) => "Add a power flag",
        }
    }
}
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ErcFinding {
    pub severity: ErcSeverity,
    pub kind: ErcKind,
    pub message: String,
    /// Where on the sheet, in micrometres: the pin's tip, the component's origin or
    /// the wire's end.
    pub at: Point,
    /// The pins it names, `U1.3`.
    pub pins: Vec<String>,
    pub fix: Option<ErcFix>,
}

/// KiCad's twelve electrical pin types, in its pin map's order.
const TYPES: [&str; 12] = [
    "input",
    "output",
    "bidirectional",
    "tri_state",
    "passive",
    "free",
    "unspecified",
    "power_in",
    "power_out",
    "open_collector",
    "open_emitter",
    "no_connect",
];
const NO_CONNECT: usize = 11;
/// A pin's electrical type as an index into [`TYPES`]. A type this build does not
/// know reads as unspecified, as KiCad reads one.
fn type_index(electrical_type: &str) -> usize {
    TYPES
        .iter()
        .position(|t| *t == electrical_type)
        .unwrap_or(6)
}
/// KiCad's default pin map: 0 allowed, 1 warning, 2 error. Symmetric.
#[rustfmt::skip]
const PIN_MAP: [[u8; 12]; 12] = [
    //          In Out Bi  3S Pas NIC UnS PwI PwO OC  OE  NC
    /* In  */ [ 0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  2],
    /* Out */ [ 0,  2,  0,  1,  0,  0,  1,  0,  2,  2,  2,  2],
    /* Bi  */ [ 0,  0,  0,  0,  0,  0,  1,  0,  1,  0,  1,  2],
    /* 3S  */ [ 0,  1,  0,  0,  0,  0,  1,  1,  2,  1,  1,  2],
    /* Pas */ [ 0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  2],
    /* NIC */ [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2],
    /* UnS */ [ 1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1,  2],
    /* PwI */ [ 0,  0,  0,  1,  0,  0,  1,  0,  0,  0,  0,  2],
    /* PwO */ [ 0,  2,  1,  2,  0,  0,  1,  0,  2,  2,  2,  2],
    /* OC  */ [ 0,  2,  0,  1,  0,  0,  1,  0,  2,  0,  0,  2],
    /* OE  */ [ 0,  2,  1,  1,  0,  0,  1,  0,  2,  0,  0,  2],
    /* NC  */ [ 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2],
];
/// Words for a type in a message: `power_in` reads "power input".
fn type_words(index: usize) -> &'static str {
    [
        "input",
        "output",
        "bidirectional",
        "tri-state",
        "passive",
        "not internally connected",
        "unspecified",
        "power input",
        "power output",
        "open collector",
        "open emitter",
        "no-connect",
    ][index]
}

impl Document {
    /// Run the electrical rule check. Errors first, then warnings, each in sheet
    /// order. A wiring diagram has no nets and so no findings.
    pub fn erc(&self) -> Vec<ErcFinding> {
        if self.kind == DocumentKind::Wiring {
            return vec![];
        }
        let netlist = self.netlist();
        let mut out = vec![];
        let tip = |component, pin: &str| {
            self.terminal_position(&Terminal {
                component,
                pin: pin.into(),
            })
            .unwrap_or_default()
        };
        let marked = |set: &[Terminal], component, pin: &str| {
            set.iter().any(|t| t.component == component && t.pin == pin)
        };
        // A pin of a gate that is not placed is on no sheet: KiCad checks it only
        // as a missing unit, below, and neither may this.
        let unplaced = |component: crate::Uuid, number: &str| {
            self.components.iter().find(|c| c.id == component).is_some_and(|c| {
                c.symbol
                    .pins
                    .iter()
                    .find(|p| p.number == number)
                    .is_some_and(|p| gate_missing(c, p.gate))
            })
        };
        // Pins whose net has no other pin, per component and gate, to fold a gate
        // with nothing joined into one finding.
        let mut open: BTreeMap<(crate::Uuid, u32), Vec<String>> = BTreeMap::new();
        for net in &netlist.nets {
            let placed = crate::Net {
                pins: net
                    .pins
                    .iter()
                    .filter(|p| !unplaced(p.component_id, &p.number))
                    .cloned()
                    .collect(),
                ..net.clone()
            };
            let net = &placed;
            let name = |p: &crate::NetPin| format!("{}.{}", p.reference, p.number);
            if net.labels.len() > 1 {
                let at = net.points[0];
                out.push(ErcFinding {
                    severity: ErcSeverity::Warning,
                    kind: ErcKind::ConflictingLabels,
                    message: format!(
                        "One net has two names: {}",
                        net.labels.iter().cloned().collect::<Vec<_>>().join(" and ")
                    ),
                    at,
                    pins: net.pins.iter().map(name).collect(),
                    fix: None,
                });
            }
            // Joined: another pin, a wire, or a label on the net.
            let joined = net.pins.len() > 1 || net.points.len() > 1 || !net.labels.is_empty();
            for p in &net.pins {
                let kind = type_index(&p.electrical_type);
                let terminal = Terminal {
                    component: p.component_id,
                    pin: p.number.clone(),
                };
                if kind == NO_CONNECT {
                    if joined {
                        out.push(ErcFinding {
                            severity: ErcSeverity::Error,
                            kind: ErcKind::NoConnectJoined,
                            message: format!("{} is a no-connect pin, but it is joined to {}", name(p), net.name),
                            at: tip(p.component_id, &p.number),
                            pins: vec![name(p)],
                            fix: None,
                        });
                    }
                    continue;
                }
                let is_marked = marked(&self.no_connects, p.component_id, &p.number);
                if is_marked {
                    if net.pins.len() > 1 {
                        out.push(ErcFinding {
                            severity: ErcSeverity::Warning,
                            kind: ErcKind::NoConnectMarkerJoined,
                            message: format!(
                                "{} is marked no-connect, but it is joined to {}",
                                name(p),
                                net.name
                            ),
                            at: tip(p.component_id, &p.number),
                            pins: vec![name(p)],
                            fix: Some(ErcFix::RemoveNoConnect(terminal)),
                        });
                    }
                    continue;
                }
                if net.pins.len() == 1 {
                    let gate = self
                        .components
                        .iter()
                        .find(|c| c.id == p.component_id)
                        .filter(|c| !c.gates.is_empty())
                        .and_then(|c| c.symbol.pins.iter().find(|q| q.number == p.number))
                        .map_or(0, |q| q.gate);
                    open.entry((p.component_id, gate))
                        .or_default()
                        .push(p.number.clone());
                }
            }
            if net.pins.len() < 2 {
                continue;
            }
            // The pin map, once per pair of types present on the net.
            let mut by_type: BTreeMap<usize, Vec<String>> = BTreeMap::new();
            for p in &net.pins {
                let kind = type_index(&p.electrical_type);
                if kind != NO_CONNECT && !marked(&self.no_connects, p.component_id, &p.number) {
                    by_type.entry(kind).or_default().push(name(p));
                }
            }
            let kinds: Vec<usize> = by_type.keys().copied().collect();
            for (i, &a) in kinds.iter().enumerate() {
                for &b in &kinds[i..] {
                    let level = PIN_MAP[a][b];
                    if level == 0 || (a == b && by_type[&a].len() < 2) {
                        continue;
                    }
                    let mut pins = by_type[&a].clone();
                    if a != b {
                        pins.extend(by_type[&b].iter().cloned());
                    }
                    let message = if a == b {
                        format!("{}: {} {} pins joined ({})", net.name, pins.len(), type_words(a), pins.join(", "))
                    } else {
                        format!(
                            "{}: {} pin {} joined to {} pin {}",
                            net.name,
                            type_words(a),
                            by_type[&a].join(", "),
                            type_words(b),
                            by_type[&b].join(", ")
                        )
                    };
                    let first = net
                        .pins
                        .iter()
                        .find(|p| name(p) == pins[0])
                        .unwrap();
                    out.push(ErcFinding {
                        severity: if level == 2 {
                            ErcSeverity::Error
                        } else {
                            ErcSeverity::Warning
                        },
                        kind: ErcKind::PinConflict,
                        message,
                        at: tip(first.component_id, &first.number),
                        pins,
                        fix: None,
                    });
                }
            }
            // A power input needs a power output or a power flag on its net.
            let power_in: Vec<&crate::NetPin> = net
                .pins
                .iter()
                .filter(|p| p.electrical_type == "power_in")
                .collect();
            let driven = net.pins.iter().any(|p| {
                p.electrical_type == "power_out" || marked(&self.power_flags, p.component_id, &p.number)
            });
            if let Some(first) = power_in.first()
                && !driven
            {
                out.push(ErcFinding {
                    severity: ErcSeverity::Error,
                    kind: ErcKind::PowerNotDriven,
                    message: format!(
                        "{}: power input {} is not driven: no power output is on the net",
                        net.name,
                        power_in.iter().map(|p| name(p)).collect::<Vec<_>>().join(", ")
                    ),
                    at: tip(first.component_id, &first.number),
                    pins: power_in.iter().map(|p| name(p)).collect(),
                    fix: Some(ErcFix::AddPowerFlag(Terminal {
                        component: first.component_id,
                        pin: first.number.clone(),
                    })),
                });
            }
        }
        // Pins joined to nothing: a whole unused gate of a part placed as gates is
        // one warning, every other pin its own error. Keyed on the GATE, not on
        // `Pin::unit`, which says which port group a pin is on.
        for ((component, gate), pins) in open {
            let Some(c) = self.components.iter().find(|c| c.id == component) else {
                continue;
            };
            let gate_pins: Vec<_> = c
                .symbol
                .pins
                .iter()
                .filter(|p| p.gate == gate && type_index(&p.electrical_type) != NO_CONNECT)
                .filter(|p| !marked(&self.no_connects, component, &p.number))
                .collect();
            let whole_gate = gate > 0
                && gate_pins.len() == pins.len()
                && !gate_pins.iter().any(|p| p.electrical_type == "power_in");
            if whole_gate {
                out.push(ErcFinding {
                    severity: ErcSeverity::Warning,
                    kind: ErcKind::UnitUnused,
                    message: format!(
                        "{} is unused: none of its pins {} is joined",
                        c.gate_reference(gate),
                        pins.join(", ")
                    ),
                    at: tip(component, &pins[0]),
                    pins: pins.iter().map(|n| format!("{}.{n}", c.reference)).collect(),
                    fix: Some(ErcFix::MarkNoConnect(
                        pins.iter()
                            .map(|pin| Terminal {
                                component,
                                pin: pin.clone(),
                            })
                            .collect(),
                    )),
                });
                continue;
            }
            for number in pins {
                let pin = c.symbol.pins.iter().find(|p| p.number == number);
                let words = pin.map_or("unspecified", |p| type_words(type_index(&p.electrical_type)));
                let pin_name = pin
                    .map(|p| p.name.as_str())
                    .filter(|n| !n.is_empty() && *n != "~" && **n != number)
                    .map(|n| format!(" ({n})"))
                    .unwrap_or_default();
                out.push(ErcFinding {
                    severity: ErcSeverity::Error,
                    kind: ErcKind::PinNotConnected,
                    message: format!("{}.{number}{pin_name}, {words}, is not connected", c.reference),
                    at: tip(component, &number),
                    pins: vec![format!("{}.{number}", c.reference)],
                    fix: Some(ErcFix::MarkNoConnect(vec![Terminal {
                        component,
                        pin: number.clone(),
                    }])),
                });
            }
        }
        // A gate of a part placed as gates that has no placement.
        for c in &self.components {
            if c.gates.is_empty() {
                continue;
            }
            for gate in (1..=c.symbol.gate_count()).filter(|&g| gate_missing(c, g)) {
                let pins: Vec<_> = c.symbol.pins.iter().filter(|p| p.gate == gate).collect();
                if pins.is_empty() {
                    continue;
                }
                let power = pins.iter().any(|p| p.electrical_type == "power_in");
                out.push(ErcFinding {
                    severity: if power { ErcSeverity::Error } else { ErcSeverity::Warning },
                    kind: ErcKind::UnitMissing,
                    message: format!(
                        "{} is not placed{}: its pins {} are on no sheet",
                        c.gate_reference(gate),
                        if power { ", so the part has no power" } else { "" },
                        pins.iter().map(|p| p.number.as_str()).collect::<Vec<_>>().join(", ")
                    ),
                    at: c.at,
                    pins: pins.iter().map(|p| format!("{}.{}", c.reference, p.number)).collect(),
                    fix: None,
                });
            }
        }
        let mut seen: BTreeMap<&str, Vec<&crate::Component>> = BTreeMap::new();
        for c in &self.components {
            seen.entry(c.reference.as_str()).or_default().push(c);
        }
        for (reference, parts) in seen {
            if parts.len() > 1 {
                out.push(ErcFinding {
                    severity: ErcSeverity::Error,
                    kind: ErcKind::DuplicateReference,
                    message: format!("{} parts are called {reference}", parts.len()),
                    at: parts[1].at,
                    pins: vec![],
                    fix: None,
                });
            }
        }
        out.extend(self.dangling_wire_ends());
        out.sort_by(|a, b| (a.severity, a.at.y, a.at.x).cmp(&(b.severity, b.at.y, b.at.x)));
        out
    }
    /// A wire end that touches no pin, no other wire, no junction and no label.
    fn dangling_wire_ends(&self) -> Vec<ErcFinding> {
        let mut anchors: BTreeSet<Point> = self.junctions.iter().copied().collect();
        anchors.extend(self.labels.iter().map(|l| l.at));
        for c in &self.components {
            anchors.extend(c.symbol.pins.iter().map(|p| c.pin_at(p)));
        }
        let mut out = vec![];
        for (i, w) in self.wires.iter().enumerate() {
            for end in [w.a, w.b] {
                let touches = anchors.contains(&end)
                    || self.wires.iter().enumerate().any(|(j, other)| {
                        j != i
                            && other
                                .points()
                                .windows(2)
                                .any(|s| crate::on_segment(end, s[0], s[1]))
                    });
                if !touches {
                    out.push(ErcFinding {
                        severity: ErcSeverity::Warning,
                        kind: ErcKind::DanglingWire,
                        message: format!(
                            "A wire ends at ({:.2}, {:.2}) mm on nothing",
                            end.x as f64 / 1000.,
                            end.y as f64 / 1000.
                        ),
                        at: end,
                        pins: vec![],
                        fix: None,
                    });
                }
            }
        }
        out
    }
    /// Make the edit a finding offers.
    pub fn apply_erc_fix(&mut self, fix: &ErcFix) {
        match fix {
            ErcFix::MarkNoConnect(pins) => {
                for t in pins {
                    if !self.no_connects.contains(t) {
                        self.no_connects.push(t.clone());
                    }
                }
            }
            ErcFix::RemoveNoConnect(t) => self.no_connects.retain(|n| n != t),
            ErcFix::AddPowerFlag(t) => {
                if !self.power_flags.contains(t) {
                    self.power_flags.push(t.clone());
                }
            }
        }
    }
}
/// Whether gate `gate` of `c` has no placement although `c` places its gates apart.
/// A component drawn whole (no gate placements) misses none.
fn gate_missing(c: &crate::Component, gate: u32) -> bool {
    gate > 0 && !c.gates.is_empty() && c.symbol.gate_count() > 1 && c.gate(gate).is_none()
}
