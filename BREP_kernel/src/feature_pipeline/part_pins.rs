//! A part's symbol PINS are its declared connection POINTS (eCAD plan
//! decision 8: pins and points are bound in the part, never on an assembly or a
//! placed component).
//!
//! # One identity: the pin label IS the point name
//!
//! eCAD names a pin with a free string (`Pin::number`: `1`, `VCC`, `A12`) and a
//! pad matches a pin by that same string. BREP used to refer to a connection
//! point by a PORT FEATURE ID (`PORT7`), which made the pin label a second,
//! separate identity that this module existed to keep paired. With the `ports`
//! block (`feature_pipeline/ports.rs`) a point's NAME is its identity — the
//! address `J1.VCC` is built out of it — so pin `VCC` and point `VCC` are one
//! name, not two that must be kept equal.
//!
//! # One symbol UNIT per port group
//!
//! A part may declare SEVERAL port groups, and `J1.VCC` and `J2.VCC` are both
//! "point VCC". The symbol says which one a pin means: eCAD's `Symbol` carries
//! `unit_count` and each [`Pin`](brep_ecad_core::Pin) carries its `unit`, which
//! is KiCad's own model for a multi-unit device. ONE UNIT MAPS TO ONE PORT
//! GROUP ([`ports::unit_map`], [`unit_groups`]), so pin `1` of unit 1 is `J1.1`
//! and pin `1` of unit 2 is `J2.1`. Two identical 8-pin headers on one part are
//! then ordinary rather than a collision, and the pin labels stay short.
//!
//! Everything below is therefore SCOPED TO A UNIT: a pin binds to the point of
//! the same name IN ITS UNIT'S GROUP, the follow calls diff one unit's pins
//! against one group's points, and a repeated name is only a clash within one
//! group. A group that maps to no unit — the piping port of a valve whose
//! symbol is only its pressure sensor — takes no pins and is left alone.
//!
//! A part that declares no ports at all reads as "unit 1 is the group a new pin
//! creates" ([`DEFAULT_PORT_NAME`]), which is the KiCad import's case.
//!
//! WAYPOINTS never enter here. A waypoint is a history feature and a
//! pass-through, never a cable end, so it has no pin and cannot be named by one.
//!
//! # Where the set lives
//!
//! A part document keeps its symbol and pads as the top-level [`SYMBOL_BLOCK`]
//! and [`PADS_BLOCK`] (eCAD's `Symbol` and `Footprint` JSON), and its connection
//! points as the `ports` block. The binding stores nothing of its own.
//!
//! The binding applies once a part carries a symbol. A 3D-only part keeps its
//! points exactly as they are, repeated names included.
//!
//! # Edits are diffed from a BASE, and an invalid state is held
//!
//! Each keystroke in a text field is an edit. Renaming pin `2` to `10` passes
//! through `` (no label) and then `1`, which repeats pin `1`. So a follow call
//! compares the edited side against `base`, the document at the START of the
//! edit's coalesced run: the snapshot BREP's history keeps for undo while one
//! coalesce key repeats. It then REWRITES the other side from `base`'s pairing,
//! addressing both sides BY INDEX while the list is the one `base` had (and by
//! name once it is not). Replaying a run from its base is idempotent, so the
//! point reads `10` whichever keystroke got there. A state that cannot be bound,
//! such as two pins of one unit sharing a label or two points of one group
//! sharing a name, returns `Err` naming it. That is a HOLD, not a refusal: the
//! user's edit stands, the other side keeps what the last bound keystroke gave
//! it, and the next keystroke is diffed from the same base again.
//!
//! Resolution is where a name must name exactly one point.
//! [`resolve_component_pin`] and [`resolve_part_pin`] REFUSE a missing or
//! repeated name by name; each takes an optional port group, which is how a
//! caller that knows the unit narrows two identical headers to one.
//!
//! [`resolve_component_pin`] names the device by its OCCURRENCE CHAIN, not by
//! one ACOMP id: a wiring diagram reaches a device nested at any depth
//! (`ACOMP7:ACOMP1`), and the endpoint's own address is what says where it sits
//! ([`endpoint_occurrence`]).

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

use serde_json::Value;

use crate::feature_pipeline::ports::{self, PortDeclaration, UnitProblem, PORTS_BLOCK};
use crate::feature_pipeline::wire_harness::WireHarnessEndpoint;
use crate::feature_pipeline::{component, PortKind, PortRecord};

/// The part document's top-level key for its schematic symbol (eCAD `Symbol`).
pub const SYMBOL_BLOCK: &str = "symbol";
/// The part document's top-level key for its footprint pads (eCAD `Footprint`).
pub const PADS_BLOCK: &str = "pads";
/// The port group a new point goes into when a part declares none yet. An
/// electronics part's pins ARE its pads, so the group it makes for them is a
/// `pcb` one.
pub const DEFAULT_PORT_NAME: &str = "Pins";
/// The purpose that group takes.
pub const DEFAULT_PORT_PURPOSE: &str = "pcb";

/// eCAD's grid step (µm): new pins are spaced by two of these.
const PIN_PITCH: i64 = 2540;
/// Where the first pin created for a point goes when the symbol has none yet:
/// its connection tip, a pin length left of the origin.
const FIRST_PIN_TIP: (i64, i64) = (-7620, 0);

// ===========================================================================
// The two sides as a document declares them
// ===========================================================================

/// One connection point as a part document declares it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeclaredPoint {
    /// The port group it belongs to (`J1`).
    pub port: String,
    /// The point name, which is what a pin binds to (`VCC`).
    pub point: String,
    /// The group's purpose (`pcb`, `wiring`, `piping`, …).
    pub purpose: String,
}

impl DeclaredPoint {
    /// Its part-local address (`J1.VCC`).
    pub fn address(&self) -> String {
        ports::address(&self.port, &self.point)
    }
}

/// One symbol pin as a part document declares it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeclaredPin {
    /// The symbol unit it belongs to, 1-based. `0` on the wire reads as `1`,
    /// which is what a symbol written before units carries.
    pub unit: u32,
    /// Its `number`, the label eCAD matches by.
    pub label: String,
}

/// Every connection point `document` declares, in block order. Only the
/// document's OWN block counts: a nested assembly's component points reach the
/// part's library entry namespaced, and they are not this part's pins.
pub fn declared_points(document: &Value) -> Vec<DeclaredPoint> {
    declarations(document)
        .into_iter()
        .flat_map(|group| {
            let (port, purpose) = (group.name.trim().to_string(), group.purpose);
            group.points.into_iter().map(move |point| DeclaredPoint {
                port: port.clone(),
                point: point.name.trim().to_string(),
                purpose: purpose.clone(),
            })
        })
        .collect()
}

/// The document's `ports` block, parsed. An unreadable block reads as none —
/// the history run reports it, and the binding has nothing to bind.
pub fn declarations(document: &Value) -> Vec<PortDeclaration> {
    document
        .get(PORTS_BLOCK)
        .cloned()
        .and_then(|block| serde_json::from_value::<Vec<PortDeclaration>>(block).ok())
        .unwrap_or_default()
}

fn port_blocks(document: &Value) -> &[Value] {
    document
        .get(PORTS_BLOCK)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

/// The symbol's pins in pin order, or `None` when the document carries no
/// symbol (the binding is off).
pub fn pins(document: &Value) -> Option<Vec<DeclaredPin>> {
    let symbol = document.get(SYMBOL_BLOCK).filter(|block| !block.is_null())?;
    Some(
        symbol
            .get("pins")
            .and_then(Value::as_array)
            .map(|pins| {
                pins.iter()
                    .map(|pin| DeclaredPin {
                        unit: pin.get("unit").and_then(Value::as_u64).unwrap_or(0).max(1) as u32,
                        label: label_of(pin),
                    })
                    .collect()
            })
            .unwrap_or_default(),
    )
}

/// The symbol's pin labels in pin order, units flattened — what a caller that
/// only wants the labels reads.
pub fn pin_labels(document: &Value) -> Option<Vec<String>> {
    Some(pins(document)?.into_iter().map(|pin| pin.label).collect())
}

/// The symbol's `unit_count`, with `0` (a symbol written before units, or one
/// serialized without the field) reading as `1`.
pub fn unit_count(document: &Value) -> u32 {
    document
        .get(SYMBOL_BLOCK)
        .and_then(|symbol| symbol.get("unit_count"))
        .and_then(Value::as_u64)
        .unwrap_or(0)
        .max(1) as u32
}

/// Which port GROUP each symbol unit binds its pins into, by name, and
/// everything about the mapping that does not hold.
///
/// The rule is [`ports::unit_map`]'s. The one thing added here is the empty
/// part: a document that declares no ports at all maps unit 1 to the
/// [`DEFAULT_PORT_NAME`] group, which the first new pin creates. That is the
/// KiCad import's case and the reason importing a symbol yields ports.
pub fn unit_groups(document: &Value) -> (BTreeMap<u32, String>, Vec<UnitProblem>) {
    let mut declarations = declarations(document);
    if declarations.is_empty() {
        declarations.push(PortDeclaration {
            name: DEFAULT_PORT_NAME.to_string(),
            purpose: DEFAULT_PORT_PURPOSE.to_string(),
            ..PortDeclaration::default()
        });
    }
    let (map, problems) = ports::unit_map(
        &declarations,
        unit_count(document),
        pins(document).is_some(),
    );
    let named = map
        .into_iter()
        .map(|(unit, index)| (unit, declarations[index].name.trim().to_string()))
        .collect();
    (named, problems)
}

/// The port group pin of unit `unit` binds into, when one is mapped.
pub fn port_group_for_unit(document: &Value, unit: u32) -> Option<String> {
    unit_groups(document).0.get(&unit.max(1)).cloned()
}

/// The footprint's pad numbers in pad order (empty = a mechanical pad).
fn pad_numbers(document: &Value) -> Vec<String> {
    document
        .get(PADS_BLOCK)
        .and_then(|block| block.get("pads"))
        .and_then(Value::as_array)
        .map(|pads| pads.iter().map(|pad| label_of(pad)).collect())
        .unwrap_or_default()
}

/// A pin's or pad's `number`: the label eCAD matches by, whatever it spells.
fn label_of(item: &Value) -> String {
    item.get("number").and_then(Value::as_str).unwrap_or_default().to_string()
}

// ---------------------------------------------------------------------------
// Slicing either side by unit / by group
// ---------------------------------------------------------------------------

/// The indices of the pins in `unit`, in pin order.
fn pins_of_unit(pins: &[DeclaredPin], unit: u32) -> Vec<usize> {
    pins.iter()
        .enumerate()
        .filter(|(_, pin)| pin.unit == unit)
        .map(|(index, _)| index)
        .collect()
}

/// The FLAT indices of the points of the group named `group`, in block order.
fn points_of_group(points: &[DeclaredPoint], group: &str) -> Vec<usize> {
    points
        .iter()
        .enumerate()
        .filter(|(_, point)| point.port == group)
        .map(|(index, _)| index)
        .collect()
}

fn labels_at(pins: &[DeclaredPin], indices: &[usize]) -> Vec<String> {
    indices.iter().map(|index| pins[*index].label.clone()).collect()
}

fn names_at(points: &[DeclaredPoint], indices: &[usize]) -> Vec<String> {
    indices.iter().map(|index| points[*index].point.clone()).collect()
}

/// Why a pin list cannot be bound: a pin with no label, or a label two pins of
/// the same unit share. eCAD's own `Document::validate` rejects both.
fn pin_list_problem(labels: &[String]) -> Option<String> {
    if labels.iter().any(String::is_empty) {
        return Some("a pin has no label".into());
    }
    let mut seen = BTreeSet::new();
    labels
        .iter()
        .find(|label| !seen.insert(label.as_str()))
        .map(|label| format!("two pins are labelled '{label}'"))
}

/// Why a point list cannot be bound: a point with no name, or a name two points
/// of one group share — a pin could not tell them apart.
fn point_list_problem(names: &[String]) -> Option<String> {
    if names.iter().any(String::is_empty) {
        return Some("a connection point has no name".into());
    }
    let mut seen = BTreeSet::new();
    names
        .iter()
        .find(|name| !seen.insert(name.as_str()))
        .map(|name| format!("two connection points are named '{name}'"))
}

/// Which point each pin pairs with, within one unit and its group: the ONE
/// point carrying its name. Indices are into the flat point list.
fn pairing(pin_labels: &[String], point_names: &[String], point_indices: &[usize]) -> Vec<Option<usize>> {
    pin_labels
        .iter()
        .map(|pin| {
            let hits: Vec<usize> = point_names
                .iter()
                .enumerate()
                .filter(|(_, name)| *name == pin)
                .map(|(local, _)| point_indices[local])
                .collect();
            match hits.as_slice() {
                [index] => Some(*index),
                _ => None,
            }
        })
        .collect()
}

// ===========================================================================
// The report: what does not pair, by name
// ===========================================================================

/// One pin and the connection point it names, by address.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinPoint {
    pub pin: String,
    /// The pin's symbol unit.
    pub unit: u32,
    pub point: String,
}

/// Something about a part's pins and points that does not pair. Each variant
/// names what is concerned; [`PinPointProblem::message`] says it in words.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "problem", rename_all = "kebab-case")]
pub enum PinPointProblem {
    /// A pin with no label.
    UnlabelledPin { unit: u32 },
    /// Two pins of one unit share a label.
    RepeatedPin { unit: u32, pin: String },
    /// No connection point in the pin's unit group carries its label.
    PinWithoutPoint { unit: u32, pin: String },
    /// Several points of one group carry one name, so a pin cannot name one.
    RepeatedPointName { name: String, points: Vec<String> },
    /// A connection point in a pin-bound group whose name no pin carries.
    PointWithoutPin { point: String },
    /// A numbered pad whose number no pin carries (an empty number is a
    /// mechanical pad and is not listed).
    PadWithoutPin { pad: String },
    /// The symbol units and the port groups disagree — see [`UnitProblem`].
    Unit(UnitProblem),
}

impl PinPointProblem {
    pub fn message(&self) -> String {
        match self {
            PinPointProblem::UnlabelledPin { unit } => format!("a pin of unit {unit} has no label"),
            PinPointProblem::RepeatedPin { unit, pin } => {
                format!("two pins of unit {unit} are labelled '{pin}'")
            }
            PinPointProblem::PinWithoutPoint { unit, pin } => {
                format!("pin '{pin}' of unit {unit} has no connection point")
            }
            PinPointProblem::RepeatedPointName { name, points } => {
                format!("{} are all named '{name}'", points.join(", "))
            }
            PinPointProblem::PointWithoutPin { point } => format!("{point} has no pin"),
            PinPointProblem::PadWithoutPin { pad } => format!("pad '{pad}' matches no pin"),
            PinPointProblem::Unit(problem) => problem.message(),
        }
    }
}

/// How a part's pins and connection points pair: every pair, then every problem.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinPointReport {
    pub pairs: Vec<PinPoint>,
    pub problems: Vec<PinPointProblem>,
}

/// How `document`'s pins and connection points pair, or `None` when it carries
/// no symbol (the binding is off; a 3D-only part's points are nobody's pins).
///
/// This is the consistency report the old `pin_port_report` was, stated against
/// names and scoped by unit. A document that arrives inconsistent — written
/// before the binding existed, replaced wholesale, or edited outside the follow
/// calls — is what it exists to describe.
pub fn pin_point_report(document: &Value) -> Option<PinPointReport> {
    let pins = pins(document)?;
    let points = declared_points(document);
    let (groups, unit_problems) = unit_groups(document);
    let mut report = PinPointReport::default();
    report
        .problems
        .extend(unit_problems.into_iter().map(PinPointProblem::Unit));

    // Every pin label carried by any unit — a pad matches a pin symbol-wide.
    let mut all_labels: BTreeSet<String> = BTreeSet::new();
    // The groups a unit binds into, so a point in an unbound group is not
    // reported as pinless.
    let bound: BTreeSet<&str> = groups.values().map(String::as_str).collect();

    for unit in units_present(&pins, &groups) {
        let pin_indices = pins_of_unit(&pins, unit);
        let labels = labels_at(&pins, &pin_indices);
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut repeated: BTreeSet<String> = BTreeSet::new();
        for label in &labels {
            if label.is_empty() {
                report.problems.push(PinPointProblem::UnlabelledPin { unit });
            } else if !seen.insert(label.clone()) && repeated.insert(label.clone()) {
                report
                    .problems
                    .push(PinPointProblem::RepeatedPin { unit, pin: label.clone() });
            }
        }
        all_labels.extend(seen.iter().cloned());

        let group = groups.get(&unit).cloned().unwrap_or_default();
        let point_indices = points_of_group(&points, &group);
        let names = names_at(&points, &point_indices);
        let mut reported: BTreeSet<String> = BTreeSet::new();
        for label in labels.iter().filter(|label| !label.is_empty()) {
            if !reported.insert(label.clone()) {
                continue;
            }
            let hits: Vec<usize> = names
                .iter()
                .enumerate()
                .filter(|(_, name)| *name == label)
                .map(|(local, _)| point_indices[local])
                .collect();
            match hits.as_slice() {
                [index] => report.pairs.push(PinPoint {
                    pin: label.clone(),
                    unit,
                    point: points[*index].address(),
                }),
                [] => report
                    .problems
                    .push(PinPointProblem::PinWithoutPoint { unit, pin: label.clone() }),
                // Reported once, below, with every point that carries it.
                _ => {}
            }
        }
        // A point of a BOUND group that no pin of that unit names.
        for index in &point_indices {
            if !seen.contains(&points[*index].point) {
                report
                    .problems
                    .push(PinPointProblem::PointWithoutPin { point: points[*index].address() });
            }
        }
    }

    // A name repeated WITHIN one group: no pin could tell the two apart.
    let mut by_group: BTreeMap<(&str, &str), Vec<String>> = BTreeMap::new();
    for point in &points {
        by_group
            .entry((point.port.as_str(), point.point.as_str()))
            .or_default()
            .push(point.address());
    }
    for ((group, name), addresses) in &by_group {
        if addresses.len() > 1 && bound.contains(group) {
            report.problems.push(PinPointProblem::RepeatedPointName {
                name: (*name).to_string(),
                points: addresses.clone(),
            });
        }
    }

    let mut pads_reported = BTreeSet::new();
    for pad in pad_numbers(document) {
        if !pad.is_empty() && !all_labels.contains(pad.as_str()) && pads_reported.insert(pad.clone())
        {
            report.problems.push(PinPointProblem::PadWithoutPin { pad });
        }
    }
    Some(report)
}

/// Every unit worth walking: the ones the symbol's pins are in, plus the ones
/// a port group claims. In ascending order.
fn units_present(pins: &[DeclaredPin], groups: &BTreeMap<u32, String>) -> Vec<u32> {
    let mut units: BTreeSet<u32> = pins.iter().map(|pin| pin.unit).collect();
    units.extend(groups.keys().copied());
    units.into_iter().collect()
}

// ===========================================================================
// Following an edit from one side to the other
// ===========================================================================

/// One change a follow call made to the side the user did not edit.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "change", rename_all = "kebab-case")]
pub enum PinPointChange {
    PointRenamed { from: String, to: String },
    PointAdded { point: String },
    PointRemoved { point: String },
    PinRelabelled { from: String, to: String },
    PinAdded { pin: String, unit: u32 },
    PinRemoved { pin: String },
    PadsRenumbered { from: String, to: String, count: usize },
    /// A pad was NOT renumbered because several units carry the old label, so
    /// the footprint cannot say which pin's pad it is (see [`renumber_pads`]).
    PadsAmbiguous { from: String, to: String, units: Vec<u32> },
}

/// The user edited the SYMBOL (or its pins' labels): carry the pin changes
/// between `base` and `after` to the connection points and pads of `after`.
///
/// Each UNIT is diffed against its own port group. Within a unit, pins are
/// matched by label first; a label in both lists is the same pin. What remains
/// pairs up in place (see [`diff_labels`]): a label that left and one that
/// arrived between the same two unchanged pins is a RENAME, and its point is
/// renamed. A label that only left removes its point. A label that only arrived
/// pairs with a point of that group already carrying it, or gets a new point in
/// it (the group is created when the part declares none). Pads follow a rename.
///
/// `Err` holds the edit (see the module docs) and leaves `after` untouched.
pub fn follow_pin_edit(base: &Value, after: &mut Value) -> Result<Vec<PinPointChange>, String> {
    let Some(pins_after) = pins(after) else {
        return Ok(Vec::new());
    };
    let pins_base = pins(base).unwrap_or_default();
    let points_base = declared_points(base);
    // The mapping is the one the EDITED document declares: a unit that gained a
    // group in this same edit binds straight away.
    let (groups, _) = unit_groups(after);

    let mut document = after.clone();
    let mut changes = Vec::new();
    // Deferred so every flat point index stays the one `base` published until
    // the renames are done (see the module docs on replaying from a base).
    let mut doomed: Vec<usize> = Vec::new();
    let mut arrivals: Vec<(String, String)> = Vec::new();

    for unit in units_present(&pins_after, &groups) {
        let after_indices = pins_of_unit(&pins_after, unit);
        let labels_after = labels_at(&pins_after, &after_indices);
        if let Some(problem) = pin_list_problem(&labels_after) {
            return Err(problem);
        }
        let Some(group) = groups.get(&unit) else {
            continue; // No group takes this unit's pins; the report says so.
        };
        let labels_base = labels_at(&pins_base, &pins_of_unit(&pins_base, unit));
        let point_indices = points_of_group(&points_base, group);
        let names_base = names_at(&points_base, &point_indices);
        let paired = pairing(&labels_base, &names_base, &point_indices);
        let (renames, removed, added) = diff_labels(&labels_base, &labels_after);

        for (index, from, to) in &renames {
            match paired[*index] {
                Some(at) => {
                    // Nothing may take a name a point this edit KEEPS already
                    // has, within the same group.
                    let current = declared_points(&document);
                    let holder = points_of_group(&current, group).into_iter().find(|other| {
                        *other != at && current[*other].point == *to
                    });
                    if let Some(holder) = holder {
                        return Err(format!(
                            "pin '{from}' cannot become '{to}': {} is already named '{to}'",
                            current[holder].address()
                        ));
                    }
                    if rename_point(base, &mut document, at, from, to) {
                        changes.push(PinPointChange::PointRenamed {
                            from: from.clone(),
                            to: to.clone(),
                        });
                    }
                }
                // The pin had no point: the new label is an arrival.
                None => arrivals.push((group.clone(), to.clone())),
            }
            // Only a pad that exists is ambiguous; a part with no footprint has
            // nothing to say here.
            let shared = units_labelled(&pins_base, from);
            if shared.len() > 1 && pad_numbers(base).iter().any(|number| number == from) {
                changes.push(PinPointChange::PadsAmbiguous {
                    from: from.clone(),
                    to: to.clone(),
                    units: shared,
                });
            } else {
                let count = renumber_pads(base, &mut document, from, to);
                if count > 0 {
                    changes.push(PinPointChange::PadsRenumbered {
                        from: from.clone(),
                        to: to.clone(),
                        count,
                    });
                }
            }
        }
        doomed.extend(removed.iter().filter_map(|index| paired[*index]));
        arrivals.extend(
            added
                .iter()
                .map(|index| (group.clone(), labels_after[*index].clone())),
        );
    }

    // Removals highest flat index first, so the earlier indices stay valid.
    doomed.sort_unstable();
    doomed.dedup();
    for at in doomed.into_iter().rev() {
        if let Some(address) = remove_point(&mut document, at) {
            changes.push(PinPointChange::PointRemoved { point: address });
        }
    }
    for (group, name) in arrivals {
        let current = declared_points(&document);
        let holders: Vec<usize> = points_of_group(&current, &group)
            .into_iter()
            .filter(|index| current[*index].point == name)
            .collect();
        match holders.as_slice() {
            [] => {
                let address = push_point(&mut document, &group, &name);
                changes.push(PinPointChange::PointAdded { point: address });
            }
            [_] => {}
            several => {
                return Err(format!(
                    "pin '{name}' names {}, which share that name",
                    several
                        .iter()
                        .map(|at| current[*at].address())
                        .collect::<Vec<_>>()
                        .join(", ")
                ));
            }
        }
    }
    *after = document;
    Ok(changes)
}

/// The user edited the CONNECTION POINTS (a point renamed, added or deleted):
/// carry the change between `base` and `after` to the pins and pads of `after`.
///
/// The two sides are symmetric: a point's name is its identity exactly as a
/// pin's label is, so this runs the same positional diff the other way, one
/// group against one unit's pins. A renamed point relabels the pin `base`
/// paired with it, a point that is gone removes its pin, and a point this edit
/// brought gets a pin below the others unless a pin of that unit already
/// carries its name. Pads follow a rename.
///
/// `Err` holds the edit (see the module docs) and leaves `after` untouched.
pub fn follow_point_edit(base: &Value, after: &mut Value) -> Result<Vec<PinPointChange>, String> {
    let Some(pins_before) = pins(after) else {
        return Ok(Vec::new());
    };
    let points_after = declared_points(after);
    let points_base = declared_points(base);
    let pins_base = pins(base).unwrap_or_default();
    let (groups, _) = unit_groups(after);

    let mut document = after.clone();
    let mut changes = Vec::new();
    // The symbol is not what this edit changed, so `after` still lists the pins
    // in `base`'s order (a run that has already relabelled one only changed its
    // label; the index is the same pin).
    let mut labels: Vec<String> = pins_before.iter().map(|pin| pin.label.clone()).collect();
    let mut doomed: Vec<usize> = Vec::new();
    let mut arrivals: Vec<(u32, String)> = Vec::new();

    for unit in units_present(&pins_before, &groups) {
        let Some(group) = groups.get(&unit) else { continue };
        let after_group = points_of_group(&points_after, group);
        let names_after = names_at(&points_after, &after_group);
        if let Some(problem) = point_list_problem(&names_after) {
            return Err(problem);
        }
        let names_base = names_at(&points_base, &points_of_group(&points_base, group));
        let unit_pins_base = pins_of_unit(&pins_base, unit);
        let (renames, removed, added) = diff_labels(&names_base, &names_after);

        // Where pin `from` of this unit sits in the CURRENT label list.
        let locate = |labels: &[String], from: &str| -> Option<usize> {
            let same_list = labels.len() == pins_base.len();
            if same_list {
                unit_pins_base
                    .iter()
                    .copied()
                    .find(|index| pins_base[*index].label == from)
            } else {
                labels.iter().position(|label| label == from)
            }
        };
        for (_, from, to) in &renames {
            let Some(at) = locate(&labels, from) else {
                arrivals.push((unit, to.clone()));
                continue;
            };
            if labels.iter().enumerate().any(|(other, label)| other != at && label == to) {
                return Err(format!(
                    "a connection point cannot become '{to}': pin '{to}' already exists"
                ));
            }
            if labels[at] != *to {
                labels[at] = to.clone();
                changes.push(PinPointChange::PinRelabelled {
                    from: from.clone(),
                    to: to.clone(),
                });
            }
            // Only a pad that exists is ambiguous; a part with no footprint has
            // nothing to say here.
            let shared = units_labelled(&pins_base, from);
            if shared.len() > 1 && pad_numbers(base).iter().any(|number| number == from) {
                changes.push(PinPointChange::PadsAmbiguous {
                    from: from.clone(),
                    to: to.clone(),
                    units: shared,
                });
            } else {
                let count = renumber_pads(base, &mut document, from, to);
                if count > 0 {
                    changes.push(PinPointChange::PadsRenumbered {
                        from: from.clone(),
                        to: to.clone(),
                        count,
                    });
                }
            }
        }
        doomed.extend(
            removed
                .iter()
                .filter_map(|index| locate(&labels, &names_base[*index])),
        );
        arrivals.extend(added.iter().map(|index| (unit, names_after[*index].clone())));
    }

    doomed.sort_unstable();
    doomed.dedup();
    for at in doomed.into_iter().rev() {
        changes.push(PinPointChange::PinRemoved { pin: labels[at].clone() });
        remove_pin(&mut document, at);
        labels.remove(at);
    }
    set_pin_labels(&mut document, &labels);
    for (unit, name) in arrivals {
        let existing = pins(&document).unwrap_or_default();
        if existing
            .iter()
            .any(|pin| pin.unit == unit && pin.label == name)
        {
            continue;
        }
        push_pin(&mut document, unit, &name);
        changes.push(PinPointChange::PinAdded { pin: name, unit });
    }
    *after = document;
    Ok(changes)
}

/// What the two versions of a part say about which point became which, and on
/// what evidence.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PointRenames {
    /// Renames a shared ANCHOR proves: both versions seat the point with the
    /// same geometry, so one IS the other whatever it is spelled.
    pub anchored: BTreeMap<String, String>,
    /// Renames read POSITIONALLY, which a delete-and-add of a point in the same
    /// place is indistinguishable from. A consumer should still APPLY them.
    pub positional: BTreeMap<String, String>,
}

impl PointRenames {
    /// Every rename, whatever the evidence — what a consumer applies.
    pub fn all(&self) -> BTreeMap<String, String> {
        let mut all = self.anchored.clone();
        all.extend(self.positional.clone());
        all
    }

    pub fn is_empty(&self) -> bool {
        self.anchored.is_empty() && self.positional.is_empty()
    }
}

/// Everything about a declared point EXCEPT its NAME: what seats it, what aims
/// it, and the descendant it maps to. A rename moves the name and leaves this
/// alone, so two versions of one part that share an anchor share a point
/// however it is spelled.
///
/// `None` for a point that carries no anchor at all. An unseated point is
/// evidence of nothing, and the display fields (`extension`, `displayLength`)
/// are left out because they say where a wire is DRAWN, not which point it is.
fn anchor(point: &ports::PortPoint) -> Option<String> {
    let seats = [
        point.point_ref.clone().unwrap_or_default(),
        point.direction_ref.clone().unwrap_or_default(),
        point.maps_to.clone().unwrap_or_default(),
        if point.transform.is_null() { String::new() } else { point.transform.to_string() },
    ];
    seats
        .iter()
        .any(|seat| !seat.is_empty())
        .then(|| seats.join("\u{1}"))
}

/// The points of one group across the two versions, paired by an anchor that is
/// UNIQUE on both sides. An anchor several points share proves nothing, so it
/// pairs nothing.
fn anchored_pairs(base: &[ports::PortPoint], after: &[ports::PortPoint]) -> Vec<(usize, usize)> {
    let unique = |points: &[ports::PortPoint]| -> BTreeMap<String, Option<usize>> {
        let mut seen: BTreeMap<String, Option<usize>> = BTreeMap::new();
        for (index, point) in points.iter().enumerate() {
            if let Some(key) = anchor(point) {
                seen.entry(key)
                    .and_modify(|slot| *slot = None)
                    .or_insert(Some(index));
            }
        }
        seen
    };
    let arrived = unique(after);
    unique(base)
        .into_iter()
        .filter_map(|(key, from)| Some((from?, (*arrived.get(&key)?)?)))
        .collect()
}

/// The points of the group named `group`, in block order. Two declarations
/// sharing a name are one group, as [`points_of_group`] reads them.
fn group_points(document: &Value, group: &str) -> Vec<ports::PortPoint> {
    declarations(document)
        .into_iter()
        .filter(|declaration| declaration.name.trim() == group)
        .flat_map(|declaration| declaration.points)
        .collect()
}

/// Which point of one part document became which in another, per group, and on
/// what evidence. There is no id behind a point any more, so this is how a
/// consumer that must carry something across a part refresh — an eCAD sheet's
/// terminals, for one — learns that `VCC` became `VDD` rather than being
/// deleted and replaced.
///
/// Two passes. The ANCHOR pass pairs points that both versions seat the same
/// way; that pairing is evidence, not a guess, and it is what makes a rename
/// survive across two SAVED versions of a part. Whatever it does not pair falls
/// to the POSITIONAL pass, which reads the remaining names the same way a
/// follow call reads one edit's worth of them — and which cannot tell a rename
/// from a delete-and-add. The two are kept apart so the consumer can say which
/// it acted on.
pub fn point_renames(base: &Value, after: &Value) -> PointRenames {
    let (points_base, points_after) = (declared_points(base), declared_points(after));
    let mut groups: Vec<&str> = points_base.iter().map(|point| point.port.as_str()).collect();
    groups.extend(points_after.iter().map(|point| point.port.as_str()));
    groups.sort_unstable();
    groups.dedup();
    let mut renames = PointRenames::default();
    for group in groups {
        let (seated_base, seated_after) = (group_points(base, group), group_points(after, group));
        let pairs = anchored_pairs(&seated_base, &seated_after);
        for (from, to) in &pairs {
            let (from, to) = (seated_base[*from].name.trim(), seated_after[*to].name.trim());
            if from != to {
                renames.anchored.insert(from.to_string(), to.to_string());
            }
        }
        let left: BTreeSet<usize> = pairs.iter().map(|(from, _)| *from).collect();
        let arrived: BTreeSet<usize> = pairs.iter().map(|(_, to)| *to).collect();
        let rest = |points: &[ports::PortPoint], paired: &BTreeSet<usize>| -> Vec<String> {
            points
                .iter()
                .enumerate()
                .filter(|(index, _)| !paired.contains(index))
                .map(|(_, point)| point.name.trim().to_string())
                .collect()
        };
        for (_, from, to) in diff_labels(&rest(&seated_base, &left), &rest(&seated_after, &arrived)).0
        {
            renames.positional.insert(from, to);
        }
    }
    renames
}

/// Split a label list's edit into renames `(base index, from, to)`, removals
/// (base indices) and arrivals (after indices).
///
/// A label in both lists is the same item wherever it moved. The labels that
/// left and the labels that arrived are grouped by how many kept labels come
/// before them. In each group they pair up in order as renames. Renaming pin 3
/// and deleting pin 1 of `[1, 2, 3]` in one change gives `[2, X]`: `1` sits
/// before the kept `2` and `3` after it, so `3 → X` pairs and `1` is removed.
///
/// What is left over then pairs across groups, in order, before anything is
/// removed or added. Shifting labels by renaming, `[A, B]` to `[B, C]`, keeps
/// `B`, and `A` (before it) and `C` (after it) sit in different groups. Pairing
/// them renames A's point `C` instead of deleting a point, and whatever
/// referenced it, only to mint a new one. Only a difference in count removes or
/// adds.
type LabelDiff = (Vec<(usize, String, String)>, Vec<usize>, Vec<usize>);

fn diff_labels(base: &[String], after: &[String]) -> LabelDiff {
    let kept: BTreeSet<&str> = base
        .iter()
        .filter(|label| after.contains(label))
        .map(String::as_str)
        .collect();
    let groups = |labels: &[String]| -> BTreeMap<usize, Vec<usize>> {
        let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        let mut before = 0;
        for (index, label) in labels.iter().enumerate() {
            if kept.contains(label.as_str()) {
                before += 1;
            } else {
                groups.entry(before).or_default().push(index);
            }
        }
        groups
    };
    let mut left = groups(base);
    let mut arrived = groups(after);
    let (mut renames, mut removed, mut added) = (Vec::new(), Vec::new(), Vec::new());
    for (group, lefts) in &mut left {
        let arrivals = arrived.remove(group).unwrap_or_default();
        let pairs = lefts.len().min(arrivals.len());
        for (from, to) in lefts.iter().zip(&arrivals) {
            renames.push((*from, base[*from].clone(), after[*to].clone()));
        }
        removed.extend(lefts.drain(pairs..));
        added.extend(arrivals.into_iter().skip(pairs));
    }
    added.extend(arrived.into_values().flatten());
    added.sort_unstable();
    let pairs = removed.len().min(added.len());
    for (from, to) in removed.iter().zip(&added) {
        renames.push((*from, base[*from].clone(), after[*to].clone()));
    }
    removed.drain(..pairs);
    added.drain(..pairs);
    (renames, removed, added)
}

// ===========================================================================
// Writing either side
// ===========================================================================

/// The point at flat index `at`, as `(block index, point index)`.
fn locate(document: &Value, at: usize) -> Option<(usize, usize)> {
    let mut seen = 0;
    for (block_index, block) in port_blocks(document).iter().enumerate() {
        let count = block.get("points").and_then(Value::as_array).map_or(0, Vec::len);
        if at < seen + count {
            return Some((block_index, at - seen));
        }
        seen += count;
    }
    None
}

fn point_mut<'a>(document: &'a mut Value, at: usize) -> Option<&'a mut Value> {
    let (block, index) = locate(document, at)?;
    document
        .get_mut(PORTS_BLOCK)?
        .as_array_mut()?
        .get_mut(block)?
        .get_mut("points")?
        .as_array_mut()?
        .get_mut(index)
}

/// Rename the point `base` had at flat index `at`. Addressed by that index
/// while the list is the one `base` had (so replaying a typing run from its
/// base finds the same point whatever intermediate name it is wearing), else by
/// the base name. Whether anything changed.
fn rename_point(base: &Value, document: &mut Value, at: usize, from: &str, to: &str) -> bool {
    let current = declared_points(document);
    let at = if current.len() == declared_points(base).len() {
        at
    } else {
        match current
            .iter()
            .enumerate()
            .filter(|(_, point)| point.point == from)
            .map(|(index, _)| index)
            .collect::<Vec<_>>()
            .as_slice()
        {
            [index] => *index,
            _ => return false,
        }
    };
    let Some(point) = point_mut(document, at).and_then(Value::as_object_mut) else {
        return false;
    };
    if point.get("name").and_then(Value::as_str) == Some(to) {
        return false;
    }
    point.insert("name".into(), Value::String(to.to_string()));
    true
}

/// Remove the point at flat index `at`; its address when one went.
fn remove_point(document: &mut Value, at: usize) -> Option<String> {
    let address = declared_points(document).get(at)?.address();
    let (block, index) = locate(document, at)?;
    let points = document
        .get_mut(PORTS_BLOCK)?
        .as_array_mut()?
        .get_mut(block)?
        .get_mut("points")?
        .as_array_mut()?;
    if index >= points.len() {
        return None;
    }
    points.remove(index);
    Some(address)
}

/// A new point named `name` in the port group `group`, creating that group
/// (with [`DEFAULT_PORT_PURPOSE`]) when the part declares no such one. Its
/// address.
fn push_point(document: &mut Value, group: &str, name: &str) -> String {
    let Some(object) = document.as_object_mut() else {
        return ports::address(group, name);
    };
    let blocks = object
        .entry(PORTS_BLOCK)
        .or_insert_with(|| Value::Array(Vec::new()));
    let Some(blocks) = blocks.as_array_mut() else {
        return ports::address(group, name);
    };
    if !blocks
        .iter()
        .any(|block| block.get("name").and_then(Value::as_str).map(str::trim) == Some(group))
    {
        blocks.push(serde_json::json!({
            "name": group, "purpose": DEFAULT_PORT_PURPOSE, "points": []
        }));
    }
    let block = blocks
        .iter_mut()
        .find(|block| block.get("name").and_then(Value::as_str).map(str::trim) == Some(group));
    let points = block
        .and_then(Value::as_object_mut)
        .map(|block| block.entry("points").or_insert_with(|| Value::Array(Vec::new())));
    if let Some(points) = points.and_then(Value::as_array_mut) {
        points.push(serde_json::json!({
            "name": name,
            "transform": { "position": [0, 0, 0], "rotationEuler": [0, 0, 0] }
        }));
    }
    ports::address(group, name)
}

fn pins_mut(document: &mut Value) -> Option<&mut Vec<Value>> {
    document.get_mut(SYMBOL_BLOCK)?.get_mut("pins")?.as_array_mut()
}

fn set_pin_labels(document: &mut Value, labels: &[String]) {
    if let Some(pins) = pins_mut(document) {
        for (pin, label) in pins.iter_mut().zip(labels) {
            if let Some(pin) = pin.as_object_mut() {
                pin.insert("number".into(), Value::String(label.clone()));
            }
        }
    }
}

fn remove_pin(document: &mut Value, index: usize) {
    if let Some(pins) = pins_mut(document) {
        if index < pins.len() {
            pins.remove(index);
        }
    }
}

/// A new passive pin for a point, in `unit`, one pin pitch below the lowest pin
/// of that unit, its connection tip in line with that unit's leftmost tip and
/// pointing right (toward the body). eCAD's sheet Y points down. The user moves
/// it in the Symbol workbench.
fn push_pin(document: &mut Value, unit: u32, label: &str) {
    let coordinate = |point: &Value, axis: &str| point.get(axis).and_then(Value::as_i64);
    let Some(symbol) = document.get_mut(SYMBOL_BLOCK).and_then(Value::as_object_mut) else {
        return;
    };
    let pins = symbol.entry("pins").or_insert_with(|| Value::Array(Vec::new()));
    let Some(pins) = pins.as_array_mut() else {
        return;
    };
    let mine = |pin: &Value| pin.get("unit").and_then(Value::as_u64).unwrap_or(0).max(1) == u64::from(unit);
    let tip_x = pins
        .iter()
        .filter(|pin| mine(pin))
        .filter_map(|pin| pin.get("at").and_then(|at| coordinate(at, "x")))
        .min()
        .unwrap_or(FIRST_PIN_TIP.0);
    let tip_y = pins
        .iter()
        .filter(|pin| mine(pin))
        .flat_map(|pin| ["at", "end"].map(|end| pin.get(end).and_then(|point| coordinate(point, "y"))))
        .flatten()
        .max()
        .map_or(FIRST_PIN_TIP.1, |lowest| lowest + PIN_PITCH);
    let mut pin = serde_json::json!({
        "hidden": false,
        "number": label,
        "name": label,
        "electrical_type": "passive",
        "at": { "x": tip_x, "y": tip_y },
        "end": { "x": tip_x + PIN_PITCH, "y": tip_y },
    });
    // Unit 1 is what an absent `unit` means, so a single-unit symbol keeps the
    // shape it had before units existed (`brep_ecad_core::Pin::unit`).
    if unit > 1 {
        if let Some(pin) = pin.as_object_mut() {
            pin.insert("unit".into(), serde_json::json!(unit));
        }
    }
    pins.push(pin);
}

/// The units of `base` whose pins carry the label `label`.
fn units_labelled(base: &[DeclaredPin], label: &str) -> Vec<u32> {
    let mut units: Vec<u32> = base
        .iter()
        .filter(|pin| pin.label == label)
        .map(|pin| pin.unit)
        .collect();
    units.sort_unstable();
    units.dedup();
    units
}

/// Renumber the pads `base` numbered `from` to `to`, by their index in `base`
/// when the pad list is the one `base` had (so replaying a typing run from its
/// base finds them again), else by the label. Returns how many changed.
///
/// Pads are matched across the WHOLE footprint, not within a unit, because that
/// is how eCAD matches one (`Pad::number` against `Pin::number`, symbol-wide).
/// When TWO units of `base` carry the label, that match cannot say whose pad it
/// is, and renumbering would take the other unit's pad along: renaming unit 2's
/// pin `1` would renumber unit 1's pad `1`. So nothing is renumbered and the
/// caller reports [`PinPointChange::PadsAmbiguous`] instead. The user renumbers
/// the pad, which is the only place that knows.
///
/// A footprint cannot hold two pads called `1` for two different pins either,
/// so a two-unit part with repeated pin labels has no correct pad mapping to
/// make — the same symbol-wide-identity seam, one layer down. This REFUSES to
/// guess at it rather than settling it.
fn renumber_pads(base: &Value, document: &mut Value, from: &str, to: &str) -> usize {
    let base_pads = pad_numbers(base);
    let same_list = pad_numbers(document).len() == base_pads.len();
    let Some(pads) = document
        .get_mut(PADS_BLOCK)
        .and_then(|block| block.get_mut("pads"))
        .and_then(Value::as_array_mut)
    else {
        return 0;
    };
    let mut count = 0;
    for (index, pad) in pads.iter_mut().enumerate() {
        let carried = if same_list { base_pads[index] == from } else { label_of(pad) == from };
        if carried && label_of(pad) != to {
            if let Some(pad) = pad.as_object_mut() {
                pad.insert("number".into(), Value::String(to.to_string()));
                count += 1;
            }
        }
    }
    count
}

// ===========================================================================
// Resolution: a placed part's pin label -> its connection point ADDRESS
// ===========================================================================

/// The connection point a pin names among `(address, port group, point name,
/// kind)` candidates. Exactly one TERMINATION must carry the name. `group`
/// narrows the search to one port group — that is how a caller that knows the
/// pin's symbol unit tells two identical headers apart; `None` searches them
/// all and refuses a name several carry.
fn resolve<'a>(
    candidates: impl Iterator<Item = (&'a str, &'a str, &'a str, PortKind)>,
    owner: &str,
    group: Option<&str>,
    pin: &str,
) -> Result<String, String> {
    let ends: Vec<&str> = candidates
        .filter(|(_, port, name, kind)| {
            *name == pin
                && *kind == PortKind::Termination
                && group.is_none_or(|group| *port == group)
        })
        .map(|(address, _, _, _)| address)
        .collect();
    let where_ = group.map_or(String::new(), |group| format!(" in port '{group}'"));
    match ends.as_slice() {
        [address] => Ok((*address).to_string()),
        [] => Err(format!("{owner} has no connection point named '{pin}'{where_}")),
        several => Err(format!(
            "{owner} has {} connection points named '{pin}'{where_} ({})",
            several.len(),
            several.join(", ")
        )),
    }
}

/// The OCCURRENCE CHAIN an endpoint address is placed under, and the
/// part-local address inside it: `ACOMP7:ACOMP1:J1.VCC` reads as
/// `("ACOMP7:ACOMP1", "J1.VCC")`, and a document's OWN point `J1.VCC` as
/// `("", "J1.VCC")`.
///
/// Read from the right, not with [`crate::split_component_namespace`]: a port
/// and a point name may not contain `:` ([`ports::RESERVED`]), so everything
/// before the LAST `:` is the chain whatever the occurrence ids are spelled
/// like. A WAYPOINT id, which is an address belonging to no port, splits the
/// same way (`ACOMP1:WP3` is unit `WP3` of `ACOMP1`).
pub fn endpoint_occurrence(address: &str) -> (&str, &str) {
    match address.rsplit_once(':') {
        Some((chain, local)) => (chain, local),
        None => ("", address),
    }
}

/// The connection-point ADDRESS that pin `pin` of the placed component
/// `occurrence` names, read from a routing report's endpoints. `occurrence` is
/// the placed component's OCCURRENCE CHAIN — `ACOMP3` for a component of this
/// assembly, `ACOMP7:ACOMP1` for one nested a level down; plan decision 5
/// resolves a Diagram reference such as `J1` to it. The endpoint's `point` is
/// the PART-LOCAL name, untouched by namespacing, so pin `VCC` of `ACOMP3` is
/// the endpoint whose point is `VCC`, and the result is its address, e.g.
/// `ACOMP3:J1.VCC`.
///
/// Candidates are matched on the endpoint's own ADDRESS
/// ([`endpoint_occurrence`]) rather than on [`WireHarnessEndpoint::component`],
/// which is the OUTERMOST occurrence alone (`SceneMap::owning_component` peels
/// one prefix) and so cannot tell `ACOMP7:ACOMP1` from `ACOMP7:ACOMP2`.
///
/// `group` is the port group the pin's symbol unit maps to
/// ([`port_group_for_unit`]); `None` means "any", and then a name several
/// groups carry is refused by name rather than guessed at.
pub fn resolve_component_pin(
    endpoints: &[WireHarnessEndpoint],
    occurrence: &str,
    group: Option<&str>,
    pin: &str,
) -> Result<String, String> {
    resolve(
        endpoints
            .iter()
            .filter(|endpoint| endpoint_occurrence(&endpoint.id).0 == occurrence)
            .map(|endpoint| {
                (
                    endpoint.id.as_str(),
                    endpoint.port.as_str(),
                    endpoint.point.as_str(),
                    endpoint.kind,
                )
            }),
        occurrence,
        group,
        pin,
    )
}

/// The part-local address that pin `pin` names among a part's own connection
/// points, as its parts-library entry carries them
/// ([`crate::PartsLibraryEntry::ports`]). `part` only names the part in a
/// refusal; `group` narrows it as [`resolve_component_pin`]'s does.
pub fn resolve_part_pin(
    ports: &BTreeMap<String, PortRecord>,
    part: &str,
    group: Option<&str>,
    pin: &str,
) -> Result<String, String> {
    resolve(
        ports.iter().map(|(address, record)| {
            (
                address.as_str(),
                record.port_name.as_str(),
                record.point_name.as_str(),
                record.kind,
            )
        }),
        &format!("part '{part}'"),
        group,
        pin,
    )
}

/// A placed component's connection points, namespaced — the shape
/// [`resolve_component_pin`] reads when a caller has the part's declarations
/// rather than a routing report (an eCAD refresh before the new parts have run).
///
/// `occurrence` is the chain the points are placed under, so a device nested a
/// level down passes `ACOMP7:ACOMP1` and its points address as
/// `ACOMP7:ACOMP1:J1.VCC`. [`WireHarnessEndpoint::component`] is the OUTERMOST
/// occurrence, which is what the scene lane's `owning_component` reports for
/// the same address.
pub fn declared_endpoints(occurrence: &str, part_document: &Value) -> Vec<WireHarnessEndpoint> {
    let outermost = occurrence.split(':').next().unwrap_or(occurrence);
    declared_points(part_document)
        .into_iter()
        .map(|declared| WireHarnessEndpoint {
            id: component::namespaced(occurrence, &declared.address()),
            port: declared.port,
            point: declared.point,
            purpose: declared.purpose,
            kind: PortKind::Termination,
            component: Some(outermost.to_string()),
        })
        .collect()
}

