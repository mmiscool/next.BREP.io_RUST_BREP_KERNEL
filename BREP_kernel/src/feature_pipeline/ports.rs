//! PORTS — a part's declared connection points, as PART DATA rather than
//! history features.
//!
//! # The block
//!
//! A part document carries `ports` as a sibling of `features`, beside the eCAD
//! `symbol` and `pads` blocks:
//!
//! ```json
//! "ports": [
//!   { "name": "J1", "purpose": "wiring",
//!     "points": [ { "name": "VCC", "transform": { … }, "pointRef": "Body_PZ" } ] }
//! ]
//! ```
//!
//! A port is a NAMED GROUP of connection points with one `purpose` — `pcb`,
//! `wiring`, `piping`, or anything else a later slice invents; the kernel
//! carries the string and never enumerates it. One part may declare several
//! ports with different purposes (a valve with a piping port and a wiring port
//! for its pressure sensor). None of this is a feature: there is no history
//! row, no dialog, no icon and no schema-catalogue entry.
//!
//! # Addressing
//!
//! A connection point is addressed `[<occurrence chain>:]<port>.<point>` —
//! `J1.VCC` in its own part, `ACOMP3:J1.VCC` once placed, `ACOMP7:ACOMP1:J1.VCC`
//! at depth. `:` is the occurrence namespace the assembly lane already speaks
//! (`component::namespaced`, peeled by `split_component_namespace`); `.`
//! separates the port group from the point. The NAME is the identity — there is
//! no second hidden id — so `:`, `.` and `/` are refused inside a port or point
//! name (`/` because the harness routing graph keys its nodes `{address}/A`).
//!
//! The address is the key in [`SceneMap::ports`], so every existing consumer —
//! `wireHarness` `from`/`to`, a spline anchor's `attachment.portRef`, a sketch
//! on the point's frame — refers to a connection point exactly as it referred
//! to a PORT feature id before, only with a name a user can read.
//!
//! # One symbol UNIT per port group
//!
//! A symbol pin binds to a connection point BY NAME (`part_pins.rs`). With
//! several port groups `J1.1` and `J2.1` are both "point 1", so the name alone
//! cannot say which. The part's SYMBOL says it instead: eCAD's `Symbol` already
//! carries `unit_count`, KiCad's own model for a multi-unit device, and ONE
//! SYMBOL UNIT MAPS TO ONE PORT GROUP. Pin `1` of unit 1 is `J1.1` and pin `1`
//! of unit 2 is `J2.1`, so pin labels stay short and two identical headers on
//! one part are ordinary rather than a collision.
//!
//! [`unit_map`] states the mapping:
//!
//! - a group may name its unit with `symbolUnit` (1-based), and that is the
//!   mapping, written down;
//! - a document with ONE group and no `symbolUnit` anywhere maps that group to
//!   unit 1. This is the ordinary single-group part, and it declares nothing;
//! - a document with SEVERAL groups binds only the groups that name a unit.
//!   Position is deliberately NOT a fallback here: a piping component declaring
//!   its piping port first and the wiring port of its pressure sensor second
//!   would otherwise bind the sensor's pins into the piping port. A group with
//!   no unit is not a schematic thing at all, which is exactly right for that
//!   piping port.
//!
//! Disagreements are REPORTED, never refused, and each names what it concerns:
//! a unit with no group ([`UnitProblem::UnitWithoutGroup`] — its pins can bind
//! to nothing), a `symbolUnit` past the symbol's `unit_count`, and two groups
//! claiming one unit. `unit_count` 0 reads as 1, which is what a symbol written
//! before units existed carries.
//!
//! # A placement is an OFFSET IN A SEAT
//!
//! A point may name geometry that places it: `pointRef` locates it,
//! `directionRef` turns it. Those two used to REPLACE the typed placement,
//! which made picking a reference the end of the conversation — there was
//! nothing left to adjust, and the `transform` beside them was dead data.
//!
//! They now supply a SEAT ([`seat_of`]): a [`Frame`] whose origin is where the
//! reference puts the point and whose **x_axis** is the direction it gives
//! (x, because a point's direction is its rotation applied to +X, the
//! convention every feature transform reads — see
//! [`Frame::from_origin_direction`]). The point's own `transform` is then read
//! IN that frame: `position` along the seat's three axes, `rotationEuler`
//! composed onto its basis. So "on this face, 2 mm off it, turned 30 degrees"
//! is one point with a reference and a small offset, and moving the face
//! carries all of it.
//!
//! A point that names NOTHING has the WORLD seat — origin at the part origin,
//! the world axes — in which an offset is an absolute placement. That is what
//! a document written before the seat existed says, and it reads exactly as it
//! did. A reference that seats the origin but implies no direction (a vertex,
//! a bare scene point) keeps the world axes too.
//!
//! What a reference may BE is the kernel's own analytic vocabulary
//! ([`crate::SelectionGeometry`], the same one an assembly constraint and a
//! PMI annotation measure against): a planar face seats on its plane along its
//! normal, a cylindrical face on its axis, a CIRCULAR edge at its CENTRE along
//! its axis, a straight edge at its chord midpoint, a vertex
//! (`{solid}@x,y,z`) at the vertex. A face or edge the analytic lane cannot
//! describe falls back to the parametric midpoint and the chord this module
//! always read.
//!
//! # Resolved AFTER history
//!
//! [`finish_history_run`] runs at the tail of every history run, after the
//! assembly solve and before the wire-harness router. That is deliberate: a
//! port point may reference ANY geometry in the part without a history-ordering
//! rule, because by the time it resolves the whole part is built. The tail is
//! authoritative: it resolves every point and overwrites whatever was there.
//!
//! There is ONE deviation, and it is a fixed two-phase, not an ordering rule.
//! [`seed_history_run`] publishes, BEFORE the walk, every point whose placement
//! is ALREADY FULLY DETERMINED — no `pointRef`, no `directionRef`, no
//! `mapsTo`, so nothing the walk builds can move it. A SPLINE in the same
//! document can then attach to it, and a sketch can sit on its frame, which is
//! the whole standalone-harness workflow. A point that DOES reference geometry
//! stays tail-only: a same-document spline attached to one reports the address
//! `unresolved` and the harness lists it as a segment problem, by name, rather
//! than quietly routing to a stale placement. In an assembly the question does
//! not arise — a placed component's points arrive with the component, during
//! the walk.
//!
//! # Assemblies declare DOWN
//!
//! An assembly declares its own ports and maps each point to a descendant's
//! point with `mapsTo`: `J1.VCC -> ACOMP2:J5.VCC`. The mapped point takes the
//! target's resolved placement, so the interface moves with the child. Depth is
//! unbounded because the target is just another address.
//!
//! # Encapsulation
//!
//! A PCB assembly is a BOUNDARY: its child components are not directly
//! wireable, and the only way to reach inside is a port the board itself
//! declares. An ordinary mechanical assembly is TRANSPARENT — an enclosure
//! holding a motor and a sensor does not hide them.
//!
//! The mark is [`document_is_boundary`]: the explicit `portBoundary` flag when
//! the document sets one, else derived — a document carrying a `pcb` block is a
//! board and therefore a boundary. Boundary is INDEPENDENT of `purpose`: a
//! terminal block on a board carries wiring-purpose points and must still be
//! declared at board level before anything may wire to it, so purpose filters
//! what KIND of connection a point is, never whether it is reachable.
//!
//! The filter is applied at EXPORT, in [`export_ports`]: what a part's library
//! entry carries is what a parent can see. A boundary part exports only the
//! points it declares itself; a transparent part exports everything its run
//! published, descendants included. `attach_component_ports` then namespaces
//! whatever arrived, and arbitrary depth falls out of the address.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{
    Axis, Env, FeatureResult, Frame, HistoryRequest, PortKind, PortRecord, SceneMap, ScenePoint,
};
use crate::{make_line, SelectionGeometry, Vec3};

/// The id of the tail's appended result (never a history feature). The scene
/// entities a declared point publishes ride it, so the display pipeline draws
/// and picks a connection point exactly as it draws a feature's curves.
pub const PORTS_FEATURE_ID: &str = "Ports";
/// The type of the appended tail result.
pub const PORTS_FEATURE_TYPE: &str = "PORTS";

/// The part document's top-level key for its declared ports.
pub const PORTS_BLOCK: &str = "ports";
/// The part document's top-level key for the explicit encapsulation flag.
pub const BOUNDARY_KEY: &str = "portBoundary";
/// What separates a port group from a point in an address.
pub const ADDRESS_SEPARATOR: char = '.';
/// Characters a port or point name may not contain: the occurrence namespace
/// separator, the address separator, and the harness graph's side separator.
pub const RESERVED: [char; 3] = [':', '.', '/'];

/// Default straight run a wire keeps from a connection point before it may bend.
pub const DEFAULT_EXTENSION: f64 = 2.0;
/// Default length of the drawn point line.
pub const DEFAULT_DISPLAY_LENGTH: f64 = 6.0;
/// The shortest drawn line — a zero-length line would publish no path.
const MIN_DISPLAY_LENGTH: f64 = 0.1;

// ===========================================================================
// The block as a document declares it
// ===========================================================================

/// One declared port: a named group of connection points with one purpose.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PortDeclaration {
    /// The group name (`J1`), part-local and unique within the document.
    #[serde(default)]
    pub name: String,
    /// What kind of connection these points are: `pcb`, `wiring`, `piping`, …
    /// Free-form and deliberately not an enum — the set is extensible.
    #[serde(default)]
    pub purpose: String,
    /// The SYMBOL UNIT whose pins bind into this group, 1-based. `None` means
    /// derive it — see [`unit_map`]. A group that maps to no unit takes no
    /// pins, which is what a piping port wants.
    #[serde(default, rename = "symbolUnit", skip_serializing_if = "Option::is_none")]
    pub symbol_unit: Option<u32>,
    #[serde(default)]
    pub points: Vec<PortPoint>,
}

/// One connection point of a declared port.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PortPoint {
    /// The point name (`VCC`), part-local and unique within its port.
    #[serde(default)]
    pub name: String,
    /// `{ position, rotationEuler }` — the placement when no reference seats
    /// it. The direction is the rotation applied to `+X`, exactly as every
    /// feature transform reads. Components may be expression strings.
    #[serde(default, skip_serializing_if = "serde_json::Value::is_null")]
    pub transform: serde_json::Value,
    /// Optional geometry that LOCATES the point: a scene point, a plane/datum
    /// frame origin, a face midpoint or an edge. Unresolved is reported, never
    /// fatal — the transform placement stands.
    #[serde(default, rename = "pointRef", skip_serializing_if = "Option::is_none")]
    pub point_ref: Option<String>,
    /// Optional geometry that sets the DIRECTION: a face normal, a frame
    /// normal, an axis or an edge chord. A `pointRef` naming a face seats the
    /// direction too when this is absent.
    #[serde(default, rename = "directionRef", skip_serializing_if = "Option::is_none")]
    pub direction_ref: Option<String>,
    #[serde(default, rename = "reverseDirection", skip_serializing_if = "is_false")]
    pub reverse_direction: bool,
    /// The straight run a wire keeps before it may bend (a spline anchor
    /// attached here takes it as its forward/backward distance).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub extension: Option<f64>,
    #[serde(default, rename = "displayLength", skip_serializing_if = "Option::is_none")]
    pub display_length: Option<f64>,
    /// ASSEMBLY declaration: the descendant point this one IS, by address
    /// (`ACOMP2:J5.VCC`). The mapped point takes the target's resolved
    /// placement, so the interface follows the child.
    #[serde(default, rename = "mapsTo", skip_serializing_if = "Option::is_none")]
    pub maps_to: Option<String>,
}

fn is_false(value: &bool) -> bool {
    !*value
}

// ===========================================================================
// Addresses
// ===========================================================================

/// The address of a point within its own part: `{port}.{point}`.
pub fn address(port: &str, point: &str) -> String {
    format!("{port}{ADDRESS_SEPARATOR}{point}")
}

/// Split a PART-LOCAL address into `(port, point)`. `None` when it carries no
/// separator — a WAYPOINT feature id, for one, which is an address in its own
/// right and belongs to no port.
pub fn split_address(address: &str) -> Option<(&str, &str)> {
    address.split_once(ADDRESS_SEPARATOR)
}

/// Why a name cannot be used, or `None` when it can.
fn name_problem(kind: &str, name: &str) -> Option<String> {
    if name.trim().is_empty() {
        return Some(format!("a {kind} has no name"));
    }
    if name.contains(RESERVED) {
        return Some(format!(
            "{kind} '{name}' contains a reserved character (':', '.' or '/')"
        ));
    }
    None
}

// ===========================================================================
// Symbol units and port groups
// ===========================================================================

/// Something about the unit-to-group mapping that does not hold. Never fatal:
/// what maps still binds, and this says what did not.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "problem", rename_all = "kebab-case")]
pub enum UnitProblem {
    /// The symbol has this unit and no port group takes it, so its pins bind
    /// to nothing.
    UnitWithoutGroup { unit: u32 },
    /// A group names a unit the symbol does not have (`0`, or past
    /// `unit_count`).
    UnitOutOfRange { port: String, unit: u32, units: u32 },
    /// Two groups name one unit; the FIRST in block order keeps it.
    UnitContested { unit: u32, ports: Vec<String> },
}

impl UnitProblem {
    pub fn message(&self) -> String {
        match self {
            UnitProblem::UnitWithoutGroup { unit } => {
                format!("symbol unit {unit} has no port group, so its pins bind to nothing")
            }
            UnitProblem::UnitOutOfRange { port, unit, units } => format!(
                "port '{port}' names symbol unit {unit}, and the symbol has {units}"
            ),
            UnitProblem::UnitContested { unit, ports } => {
                format!("ports {} all name symbol unit {unit}", ports.join(", "))
            }
        }
    }
}

/// Which port group each symbol unit binds its pins into, and everything about
/// the mapping that does not hold. The map is `unit (1-based) -> index into
/// `declarations``; a unit absent from it binds no pins.
///
/// `unit_count` is the symbol's, and `0` reads as `1`. The rule is in this
/// module's docs; in short, an explicit `symbolUnit` decides, one group with
/// none takes unit 1, and several groups with none bind nothing.
///
/// `has_symbol` is whether the document carries a symbol at all. A 3D-only part
/// has no units, so it reports no missing ones.
pub fn unit_map(
    declarations: &[PortDeclaration],
    unit_count: u32,
    has_symbol: bool,
) -> (BTreeMap<u32, usize>, Vec<UnitProblem>) {
    let units = unit_count.max(1);
    let mut map: BTreeMap<u32, usize> = BTreeMap::new();
    let mut problems = Vec::new();
    let mut contested: BTreeMap<u32, Vec<String>> = BTreeMap::new();

    let explicit: Vec<(usize, u32)> = declarations
        .iter()
        .enumerate()
        .filter_map(|(index, group)| group.symbol_unit.map(|unit| (index, unit)))
        .collect();
    for (index, unit) in &explicit {
        let port = declarations[*index].name.trim().to_string();
        if *unit == 0 || *unit > units {
            problems.push(UnitProblem::UnitOutOfRange { port, unit: *unit, units });
            continue;
        }
        match map.get(unit) {
            Some(first) => contested
                .entry(*unit)
                .or_insert_with(|| vec![declarations[*first].name.trim().to_string()])
                .push(port),
            None => {
                map.insert(*unit, *index);
            }
        }
    }
    for (unit, ports) in contested {
        problems.push(UnitProblem::UnitContested { unit, ports });
    }
    // The ordinary part: one group, nothing written down, unit 1.
    if explicit.is_empty() && declarations.len() == 1 {
        map.insert(1, 0);
    }
    if has_symbol {
        for unit in 1..=units {
            if !map.contains_key(&unit) {
                problems.push(UnitProblem::UnitWithoutGroup { unit });
            }
        }
    }
    (map, problems)
}

// ===========================================================================
// The tail's report
// ===========================================================================

/// A [`Frame`] as the ports REPORT carries it: four plain triples rather than
/// [`Vec3`]s, because a report is compared and serialized (a `Vec3` carries no
/// `PartialEq`, by choice, and this is the shape the app and the verifier
/// read).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PortSeat {
    pub origin: [f64; 3],
    /// The seat's x axis, which is the direction the reference gives — see
    /// [`Frame::from_origin_direction`].
    pub x: [f64; 3],
    pub y: [f64; 3],
    pub z: [f64; 3],
}

impl PortSeat {
    pub fn to_frame(self) -> Frame {
        let v = |a: [f64; 3]| Vec3::new(a[0], a[1], a[2]);
        Frame {
            origin: v(self.origin),
            x_axis: v(self.x),
            y_axis: v(self.y),
            z_axis: v(self.z),
        }
    }
}

impl From<Frame> for PortSeat {
    fn from(frame: Frame) -> Self {
        let a = |v: Vec3| [v.x, v.y, v.z];
        PortSeat {
            origin: a(frame.origin),
            x: a(frame.x_axis),
            y: a(frame.y_axis),
            z: a(frame.z_axis),
        }
    }
}

/// One resolved connection point, as the tail publishes it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PortPointRow {
    /// The part-local address (`J1.VCC`) — the [`SceneMap::ports`] key.
    pub address: String,
    pub port: String,
    pub point: String,
    pub purpose: String,
    /// The descendant address this point maps to, when it declares one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub maps_to: Option<String>,
    /// The frame this point's `transform` was read as an offset IN
    /// ([`seat_of`]) — the world frame for a point that names no reference.
    ///
    /// Report-only, and deliberately not on [`PortRecord`]: a seat is what the
    /// last RUN made of the references, not something the document carries or
    /// a library entry exports. The editor needs it to put a gizmo on the
    /// point, because dragging one in world space has to be written back as an
    /// offset in exactly this frame.
    pub seat: PortSeat,
    /// Where the point resolved, in world space (`record.point`).
    pub position: [f64; 3],
    /// The unit direction it resolved facing (`record.direction`).
    pub direction: [f64; 3],
}

/// What the ports tail resolved, and everything about the block that does not
/// hold — a name it refused, a reference that resolved to nothing, a mapping
/// whose target is not in the scene. Never fatal: a problem is reported and the
/// point still publishes at whatever placement could be worked out, so nothing
/// downstream vanishes (the same contract rule the features follow).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PortsReport {
    /// Whether this document encapsulates its children ([`document_is_boundary`]).
    #[serde(default)]
    pub boundary: bool,
    pub points: Vec<PortPointRow>,
    pub problems: Vec<String>,
}

// ===========================================================================
// Encapsulation
// ===========================================================================

/// Whether a DOCUMENT hides its children behind its declared ports.
///
/// The explicit `portBoundary` flag decides when the document sets one. With no
/// flag it is derived: a document carrying a `pcb` block is a board, and a
/// board is a boundary by construction. Derivation alone could not mark a
/// non-PCB enclosure; a flag alone would have to be set on every board by hand,
/// which is exactly the case the user said holds "by construction".
pub fn document_is_boundary(document: &serde_json::Value) -> bool {
    match document.get(BOUNDARY_KEY).and_then(serde_json::Value::as_bool) {
        Some(explicit) => explicit,
        None => document.get("pcb").is_some_and(|block| !block.is_null()),
    }
}

/// The same question for a parsed request (the tail's lane).
pub fn request_is_boundary(request: &HistoryRequest) -> bool {
    request.port_boundary.unwrap_or(request.pcb_present)
}

/// What a part's library entry carries out of its own run — the encapsulation
/// filter, applied ONCE, here.
///
/// A TRANSPARENT part exports every port record its run published: its own
/// declared points and, namespaced, every point its children brought. A
/// BOUNDARY part exports only the points it declares itself, so a parent can
/// reach its connectors and nothing else.
pub fn export_ports(
    scene_ports: &BTreeMap<String, PortRecord>,
    declared: &[String],
    boundary: bool,
) -> BTreeMap<String, PortRecord> {
    if !boundary {
        return scene_ports.clone();
    }
    let declared: BTreeSet<&str> = declared.iter().map(String::as_str).collect();
    scene_ports
        .iter()
        .filter(|(address, _)| declared.contains(address.as_str()))
        .map(|(address, record)| (address.clone(), record.clone()))
        .collect()
}

// ===========================================================================
// Publishing one point's scene entities
// ===========================================================================

/// The drawn line for a connection point: a termination's wire leaves along the
/// direction; a waypoint's wire passes through, so its line straddles the base
/// point.
pub fn port_line(record: &PortRecord) -> (Vec3, Vec3) {
    let point = record.point;
    let direction = record.direction;
    let length = record.display_length;
    match record.kind {
        PortKind::Termination => (point, point.add(direction.scale(length))),
        PortKind::Waypoint => (
            point.sub(direction.scale(length * 0.5)),
            point.add(direction.scale(length * 0.5)),
        ),
    }
}

/// Publish one connection point's scene entities under `address`: the record
/// (`ports[address]`), its base point (`{address}:Base`), its axis and its
/// frame (both `address`) and its drawn line (`{address}:PortLine`). A sketch
/// on the frame is a connector face square to the wire.
///
/// The `Vec` side-channel form, for a FEATURE result (WAYPOINT, and the ACOMP
/// that re-publishes a placed part's points).
pub fn publish_port(
    result: &mut crate::feature_pipeline::FeatureResult,
    address: &str,
    record: PortRecord,
) -> Result<(), String> {
    let (start, end) = port_line(&record);
    result
        .paths
        .push((format!("{address}:PortLine"), vec![make_line(start, end)?]));
    result
        .points
        .push((format!("{address}:Base"), ScenePoint::model(record.point)));
    result.axes.push((
        address.to_string(),
        Axis { point: record.point, direction: record.direction },
    ));
    result.frames.push((
        address.to_string(),
        Frame::from_origin_normal(record.point, record.direction)?,
    ));
    result.ports.push((address.to_string(), record));
    Ok(())
}

/// The same entities written straight into a scene — the ports tail and the
/// component re-pose lane, which moves a placed part's points without
/// re-running its feature.
pub fn place_scene_port(
    scene: &mut SceneMap,
    address: &str,
    record: PortRecord,
) -> Result<(), String> {
    let (start, end) = port_line(&record);
    scene
        .paths
        .insert(format!("{address}:PortLine"), vec![make_line(start, end)?]);
    scene
        .points
        .insert(format!("{address}:Base"), ScenePoint::model(record.point));
    scene.axes.insert(
        address.to_string(),
        Axis { point: record.point, direction: record.direction },
    );
    scene.frames.insert(
        address.to_string(),
        Frame::from_origin_normal(record.point, record.direction)?,
    );
    scene.ports.insert(address.to_string(), record);
    Ok(())
}

// ===========================================================================
// The SEAT — what a reference resolves to, and the frame a placement sits in
// ===========================================================================

/// Where a reference sits, and the direction it implies when it has one.
///
/// The vocabulary is the kernel's own analytic one ([`SelectionGeometry`]), so
/// a connection point seats on exactly what an assembly constraint and a PMI
/// annotation measure against:
///
/// | picked | seats at | points along |
/// |---|---|---|
/// | a planar face | the face's boundary-AABB centre on its plane | its outward normal |
/// | a cylindrical / conical face (a bore) | a point on its axis | its axis |
/// | a spherical face | the centre | — |
/// | a CIRCULAR edge (a hole rim) | the **centre** | its axis |
/// | a straight edge | the chord midpoint | the chord |
/// | a vertex (`{solid}@x,y,z`) | the vertex | — |
/// | a scene point, a datum/plane frame, an axis | its own origin | its own normal / direction |
///
/// The scene-native names (points, frames, axes) are tried FIRST and keep the
/// meaning they always had. Everything else goes through the analytic
/// resolver, and a face or edge it calls unsupported (a freeform patch, a
/// spline edge) falls back to what this module read before: a face's
/// parametric midpoint and normal, an edge's start and chord.
fn resolve_seat_reference(scene: &SceneMap, name: &str) -> Result<(Vec3, Option<Vec3>), String> {
    if let Some(position) = scene.resolve_point(name) {
        return Ok((position, None));
    }
    if let Some(frame) = scene.resolve_frame(name) {
        return Ok((frame.origin, Some(frame.z_axis)));
    }
    if let Some(axis) = scene.resolve_axis(name) {
        return Ok((axis.point, Some(axis.direction)));
    }
    match analytic_reference(scene, name) {
        Some(Ok(geometry)) => Ok(seat_of_geometry(&geometry)),
        Some(Err(problem)) => Err(problem),
        None => Err(format!("'{name}' resolves to no geometry")),
    }
}

/// [`resolve_seat_reference`]'s direction half: the same vocabulary, keeping
/// only what implies a direction. A reference that implies none (a vertex, a
/// sphere's centre, a bare scene point) is refused BY NAME rather than
/// silently leaving the direction where it was.
fn resolve_seat_direction(scene: &SceneMap, name: &str) -> Result<Vec3, String> {
    if let Some(frame) = scene.resolve_frame(name) {
        return Ok(frame.z_axis);
    }
    if let Some(axis) = scene.resolve_axis(name) {
        return Ok(axis.direction);
    }
    if scene.resolve_point(name).is_some() {
        return Err(format!("'{name}' is a point, which implies no direction"));
    }
    match analytic_reference(scene, name) {
        Some(Ok(geometry)) => seat_of_geometry(&geometry)
            .1
            .ok_or_else(|| format!("'{name}' implies no direction")),
        Some(Err(problem)) => Err(problem),
        None => Err(format!("'{name}' resolves to no geometry")),
    }
}

/// One analytic entity's seat: where it sits and the direction it implies.
fn seat_of_geometry(geometry: &SelectionGeometry) -> (Vec3, Option<Vec3>) {
    match geometry {
        SelectionGeometry::Plane { origin, normal } => (*origin, Some(*normal)),
        SelectionGeometry::Axis { origin, direction, .. } => (*origin, Some(*direction)),
        SelectionGeometry::Sphere { center, .. } => (*center, None),
        SelectionGeometry::Circle { center, axis, .. } => (*center, Some(*axis)),
        SelectionGeometry::Line { origin, direction } => (*origin, Some(*direction)),
        SelectionGeometry::Point { position } => (*position, None),
    }
}

/// `name` as an ANALYTIC entity — a face, an edge, or a `{solid}@x,y,z` vertex
/// reference — or `None` when the scene holds no such thing under that name.
///
/// `Some(Err(..))` is a name that DID resolve to topology the analytic lane
/// cannot describe. A face or edge in that state falls back to this module's
/// older reading rather than refusing the point: a freeform seat is worse than
/// an analytic one and much better than none.
fn analytic_reference(
    scene: &SceneMap,
    name: &str,
) -> Option<Result<SelectionGeometry, String>> {
    // A VERTEX carries no kernel name, so the viewport references one by its
    // owning solid and world position — the same form PMI writes, resolved by
    // the same helper (nearest topology vertex, snap-gated).
    if name.contains('@') {
        return Some(
            crate::feature_pipeline::pmi::resolve::resolve_reference(scene, name)
                .map_err(|error| error.to_string()),
        );
    }
    if let Some(face) = scene.resolve_face(name) {
        let analytic = crate::with_registered_solid_str(face.handle, |solid| {
            Ok(crate::resolve_face_selection(solid, face.face_id))
        });
        return Some(match analytic {
            Ok(Ok(geometry)) => Ok(geometry),
            // Unsupported / failed: the parametric midpoint and normal, which
            // is what a `pointRef` on a face has always meant here.
            _ => common::face_point_normal(face).map(|(point, normal)| {
                SelectionGeometry::Plane { origin: point, normal }
            }),
        });
    }
    if let Some(edge) = scene.resolve_edge(name) {
        let analytic = crate::with_registered_solid_str(edge.handle, |solid| {
            Ok(crate::resolve_edge_selection(solid, edge.edge_id))
        });
        return Some(match analytic {
            Ok(Ok(geometry)) => Ok(geometry),
            // A freeform edge: its start and chord, as before. A CLOSED one
            // has a zero chord and refuses itself here rather than failing the
            // whole point — which is what a circle used to do, before the
            // analytic lane above started answering with its centre.
            _ => common::edge_axis(edge).map(|axis| SelectionGeometry::Line {
                origin: axis.point,
                direction: axis.direction,
            }),
        });
    }
    None
}

/// The FRAME a connection point's placement is an offset in.
///
/// Its origin is where `pointRef` seats the point (the part origin when there
/// is none) and its **x_axis** is the direction that seats it: `directionRef`
/// when the point names one, else whatever `pointRef` implied, else +X. The
/// point's own `transform` is then read IN this frame — `position` along its
/// three axes, `rotationEuler` composed onto its basis — so a point seated on
/// a face can be nudged 2 mm off it and turned 30 degrees on it without the
/// reference being lost.
///
/// With NO reference at all the seat is the world frame, and a local offset in
/// the world frame is an absolute placement: that is the case every document
/// written before the seat existed is in, and it reads exactly as it did.
///
/// Problems are collected, never fatal. A reference that resolves to nothing
/// leaves that half of the seat at its default and says so by name.
pub fn seat_of(
    point: &PortPoint,
    scene: &SceneMap,
) -> (Frame, Vec<String>) {
    let mut problems = Vec::new();
    let mut origin = Vec3::new(0.0, 0.0, 0.0);
    // `None` until a reference SUPPLIES one. A seated origin with no direction
    // — a vertex, a bare scene point — keeps the world axes, so the point's
    // own rotation still means what it means everywhere else in the document.
    let mut direction: Option<Vec3> = None;

    if let Some(reference) = trimmed(point.point_ref.as_deref()) {
        match resolve_seat_reference(scene, reference) {
            Ok((located, implied)) => {
                origin = located;
                if let Some(implied) = implied {
                    direction = Some(implied);
                }
            }
            Err(problem) => problems.push(problem),
        }
    }
    if let Some(reference) = trimmed(point.direction_ref.as_deref()) {
        match resolve_seat_direction(scene, reference) {
            Ok(resolved) => direction = Some(resolved),
            Err(problem) => problems.push(problem),
        }
    }
    // No direction, or one too degenerate to make a basis from: the WORLD axes
    // at whatever origin was seated. With no references at all that is the
    // world frame exactly, in which an offset IS an absolute placement — which
    // is what every document written before the seat existed says.
    let seat = match direction {
        Some(direction) => Frame::from_origin_direction(origin, direction)
            .unwrap_or_else(|_| Frame { origin, ..world_seat() }),
        None => Frame { origin, ..world_seat() },
    };
    (seat, problems)
}

/// A local offset carried into world space by `seat`.
pub fn seat_point(seat: &Frame, local: Vec3) -> Vec3 {
    seat.origin
        .add(seat.x_axis.scale(local.x))
        .add(seat.y_axis.scale(local.y))
        .add(seat.z_axis.scale(local.z))
}

/// A world position read back as an offset in `seat` — the inverse of
/// [`seat_point`], which is what a gizmo drag writes. The basis is orthonormal,
/// so the inverse is the transpose.
pub fn unseat_point(seat: &Frame, world: Vec3) -> Vec3 {
    unseat_vector(seat, world.sub(seat.origin))
}

/// A local DIRECTION carried into world space by `seat` (no translation).
pub fn seat_vector(seat: &Frame, local: Vec3) -> Vec3 {
    seat.x_axis
        .scale(local.x)
        .add(seat.y_axis.scale(local.y))
        .add(seat.z_axis.scale(local.z))
}

/// A world direction read back in `seat`'s basis.
pub fn unseat_vector(seat: &Frame, world: Vec3) -> Vec3 {
    Vec3::new(
        world.dot(seat.x_axis),
        world.dot(seat.y_axis),
        world.dot(seat.z_axis),
    )
}

/// The world direction a point with this seat and this rotation faces: the
/// rotation applied to +X (every feature transform's convention), carried
/// through the seat's basis. With the world seat this is the rotated +X
/// itself.
pub fn seated_direction(seat: &Frame, rotation_deg: [f64; 3]) -> Vec3 {
    let local = crate::feature_pipeline::features::datum::rotate_euler_xyz(
        Vec3::new(1.0, 0.0, 0.0),
        [
            rotation_deg[0].to_radians(),
            rotation_deg[1].to_radians(),
            rotation_deg[2].to_radians(),
        ],
    );
    seat_vector(seat, local)
}

/// A non-empty trimmed reference, or `None`.
fn trimmed(value: Option<&str>) -> Option<&str> {
    value.map(str::trim).filter(|text| !text.is_empty())
}

/// A `directionRef`'s unit direction, or `None` when the name resolves to
/// nothing that implies one. The WAYPOINT feature's lane — a waypoint has no
/// seat (it is a feature with its own transform), so it takes the direction
/// alone and lists an unresolved name in its own report.
pub fn resolve_direction(scene: &SceneMap, name: &str) -> Result<Option<Vec3>, String> {
    Ok(resolve_seat_direction(scene, name).ok())
}

// ===========================================================================
// The tail
// ===========================================================================

/// Publish, BEFORE the history walk, every declared point whose placement is
/// already fully determined — see the module doc. Returns each published scene
/// name with the CONTENT VERSION of the declaration behind it, so the
/// incremental cache invalidates a feature that consumed the point when the
/// declaration moves (without this a spline attached to a point would replay
/// from cache at the point's old placement).
pub fn seed_history_run(
    request: &HistoryRequest,
    scene: &mut SceneMap,
    env: &Env,
) -> Vec<(String, u64)> {
    let mut seeded = Vec::new();
    let mut ports_seen: BTreeSet<&str> = BTreeSet::new();
    for declaration in &request.ports {
        let port = declaration.name.trim();
        if name_problem("port", port).is_some() || !ports_seen.insert(port) {
            continue;
        }
        let mut points_seen: BTreeSet<&str> = BTreeSet::new();
        for point in &declaration.points {
            let name = point.name.trim();
            if name_problem("point", name).is_some() || !points_seen.insert(name) {
                continue;
            }
            if point.point_ref.is_some() || point.direction_ref.is_some() || point.maps_to.is_some()
            {
                continue;
            }
            let Ok((record, _, _)) =
                resolve_point(point, port, name, &declaration.purpose, scene, env)
            else {
                continue;
            };
            let address = address(port, name);
            if place_scene_port(scene, &address, record).is_err() {
                continue;
            }
            let version = declaration_version(port, &declaration.purpose, point);
            for name in [
                address.clone(),
                format!("{address}:Base"),
                format!("{address}:PortLine"),
            ] {
                seeded.push((name, version));
            }
        }
    }
    seeded
}

/// The content version of one declared point: its port, its purpose and the
/// point itself. Two runs that declare the same point the same way produce the
/// same number, and any edit to it produces a different one.
fn declaration_version(port: &str, purpose: &str, point: &PortPoint) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    port.hash(&mut hasher);
    purpose.hash(&mut hasher);
    serde_json::to_string(point).unwrap_or_default().hash(&mut hasher);
    hasher.finish()
}

/// What the ports tail produced: its report, and the appended result the
/// display pipeline consumes.
#[derive(Debug, Default)]
pub struct PortsOutcome {
    pub report: Option<PortsReport>,
    pub result: Option<FeatureResult>,
}

/// Resolve the request's `ports` block against the FINISHED scene and publish
/// every point. Runs after the assembly solve (so a mapped point reads its
/// child's posed placement) and before the wire-harness router (which routes
/// over what this published). Empty when the document declares no ports.
pub fn finish_history_run(
    request: &HistoryRequest,
    scene: &mut SceneMap,
    env: &Env,
) -> PortsOutcome {
    if request.ports.is_empty() {
        return PortsOutcome::default();
    }
    let mut result = FeatureResult::pass_through(
        PORTS_FEATURE_ID.to_string(),
        PORTS_FEATURE_TYPE.to_string(),
    );
    let mut report = PortsReport {
        boundary: request_is_boundary(request),
        ..PortsReport::default()
    };
    let mut ports_seen: BTreeSet<&str> = BTreeSet::new();

    for declaration in &request.ports {
        let port = declaration.name.trim();
        if let Some(problem) = name_problem("port", port) {
            report.problems.push(problem);
            continue;
        }
        if !ports_seen.insert(port) {
            report.problems.push(format!("two ports are named '{port}'"));
            continue;
        }
        let mut points_seen: BTreeSet<&str> = BTreeSet::new();
        for point in &declaration.points {
            let name = point.name.trim();
            if let Some(problem) = name_problem(&format!("point of port '{port}'"), name) {
                report.problems.push(problem);
                continue;
            }
            if !points_seen.insert(name) {
                report
                    .problems
                    .push(format!("port '{port}' has two points named '{name}'"));
                continue;
            }
            let address = address(port, name);
            match resolve_point(point, port, name, &declaration.purpose, scene, env) {
                Ok((record, seat, problems)) => {
                    report.problems.extend(problems);
                    let (position, direction) = (record.point, record.direction);
                    if let Err(error) = publish_port(&mut result, &address, record) {
                        report.problems.push(format!("{address}: {error}"));
                        continue;
                    }
                    report.points.push(PortPointRow {
                        address,
                        port: port.to_string(),
                        point: name.to_string(),
                        purpose: declaration.purpose.clone(),
                        maps_to: point.maps_to.clone(),
                        seat: seat.into(),
                        position: [position.x, position.y, position.z],
                        direction: [direction.x, direction.y, direction.z],
                    });
                }
                Err(error) => report.problems.push(format!("{address}: {error}")),
            }
        }
    }
    scene.apply(&result);
    PortsOutcome { report: Some(report), result: Some(result) }
}

/// One point's record and its seat, plus whatever about it did not hold. A
/// reference that resolves to nothing is a problem, never an error: the point
/// still publishes, at whatever placement could be worked out.
///
/// The placement rule, in one sentence: **a point's `transform` is an offset in
/// its seat** ([`seat_of`]) — its `position` along the seat's three axes, its
/// `rotationEuler` composed onto the seat's basis. A point with no reference
/// has the WORLD seat, where an offset is an absolute placement, so every
/// document written before the seat existed reads exactly as it did.
fn resolve_point(
    point: &PortPoint,
    port: &str,
    name: &str,
    purpose: &str,
    scene: &SceneMap,
    env: &Env,
) -> Result<(PortRecord, Frame, Vec<String>), String> {
    let address = address(port, name);
    let mut problems = Vec::new();

    // A mapped point IS its target: the assembly declares the interface, the
    // descendant holds the geometry. It has no seat of its own — its placement
    // is the target's, and a gizmo on one would be editing the wrong document.
    if let Some(target) = point.maps_to.as_deref().map(str::trim).filter(|t| !t.is_empty()) {
        match scene.resolve_port(target) {
            Some(mapped) => {
                let seat = Frame::from_origin_direction(mapped.point, mapped.direction)
                    .unwrap_or_else(|_| world_seat());
                return Ok((
                    PortRecord {
                        point: mapped.point,
                        direction: mapped.direction,
                        kind: PortKind::Termination,
                        extension: point.extension.unwrap_or(mapped.extension).max(0.0),
                        display_length: point
                            .display_length
                            .unwrap_or(mapped.display_length)
                            .max(MIN_DISPLAY_LENGTH),
                        port_name: port.to_string(),
                        point_name: name.to_string(),
                        purpose: purpose.to_string(),
                    },
                    seat,
                    problems,
                ));
            }
            None => problems.push(format!(
                "{address} maps to '{target}', which is not a connection point of this document"
            )),
        }
    }

    let position = common::vec3_from_value(
        env,
        point.transform.get("position"),
        &format!("{address}.transform.position"),
        [0.0, 0.0, 0.0],
    )?;
    let rotation = common::vec3_from_value(
        env,
        point.transform.get("rotationEuler"),
        &format!("{address}.transform.rotationEuler"),
        [0.0, 0.0, 0.0],
    )?;

    let (seat, seat_problems) = seat_of(point, scene);
    problems.extend(
        seat_problems
            .into_iter()
            .map(|problem| format!("{address}: {problem}")),
    );

    let base = seat_point(&seat, Vec3::new(position[0], position[1], position[2]));
    let mut direction = seated_direction(&seat, rotation)
        .normalized()
        .map_err(|_| "direction is a zero vector".to_string())?;
    if point.reverse_direction {
        direction = direction.scale(-1.0);
    }
    Ok((
        PortRecord {
            point: base,
            direction,
            kind: PortKind::Termination,
            extension: point.extension.unwrap_or(DEFAULT_EXTENSION).max(0.0),
            display_length: point
                .display_length
                .unwrap_or(DEFAULT_DISPLAY_LENGTH)
                .max(MIN_DISPLAY_LENGTH),
            port_name: port.to_string(),
            point_name: name.to_string(),
            purpose: purpose.to_string(),
        },
        seat,
        problems,
    ))
}

/// The seat of a point that names no reference: the world frame, in which an
/// offset is an absolute placement.
pub fn world_seat() -> Frame {
    Frame {
        origin: Vec3::new(0.0, 0.0, 0.0),
        x_axis: Vec3::new(1.0, 0.0, 0.0),
        y_axis: Vec3::new(0.0, 1.0, 0.0),
        z_axis: Vec3::new(0.0, 0.0, 1.0),
    }
}

