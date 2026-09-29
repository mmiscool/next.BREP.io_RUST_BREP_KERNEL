//! Deleting a whole BLEND NETWORK — every strip, vertex blend and mitre a
//! fillet left around a feature — as ONE question.
//!
//! Round the edges where a gusset meets a floor and a wall and the fillet
//! leaves a ring of faces around the gusset's foot: a strip along each of the
//! six edges, a spherical vertex blend where three of them meet at each corner,
//! and MITRES where a strip along the gusset's side runs into the strip along
//! its sloping top. Selecting that ring and asking for it gone is "un-round the
//! gusset", and it is the corner blend's question ([`corner_heal`]) with more
//! than one corner and with strips whose far end is not a survivor.
//!
//! One face at a time it is ill-posed for the corner lane's reason: a strip's
//! walls re-intersect in its sharp edge, and whatever the strip runs into at a
//! vertex blend or a mitre is itself about to go. The corner lane answers one
//! vertex blend and three strips whose far ends are caps. Here most ends are
//! not caps, so the set has to be read as the network it is.
//!
//! ## The construction
//!
//! Read off the network's own topology and the tangency that defines a blend:
//!
//! * a boundary edge of a selected face is a RIM when the survivor across it is
//!   TANGENT to the selected face there — that is what a blend's rim is. Every
//!   other boundary edge — a vertex blend's edges, a mitre between two strips,
//!   the edge a strip ends on at a cap face — is an END edge;
//! * a STRIP has exactly two rims, on two distinct survivors: its WALLS. A
//!   selected face with no rim at all must border only selected faces (a
//!   vertex blend enclosed by its strips), and is all end edges;
//! * a JUNCTION is a connected cluster of end edges. Everything in a junction
//!   shrinks to ONE vertex of the unblended solid — a vertex blend, a mitre and
//!   the cap edges beside it, a strip's cap on an end wall — and every strip
//!   runs between exactly two junctions.
//!
//! Then, in closed form, because every face the network touches is a PLANE:
//! each junction's vertex is where the planes of every survivor meeting at it
//! intersect, and all of them must contain it — the corner lane's "three lines
//! name one vertex" gate, for any number of planes. Each strip becomes the
//! sharp edge between its two junction vertices, which lies on both its walls
//! by that same gate.
//!
//! The topology follows the corner lane's: every network vertex collapses onto
//! its junction's vertex, every surviving edge that ended on a collapsed vertex
//! is widened along its own line, each wall swaps its rim for the sharp edge,
//! every survivor drops the end edges it wore, and every touched plane is
//! re-trimmed and its pcurves refit. The Euler characteristic is required to
//! be unchanged — un-rounding does not add or remove a handle.
//!
//! ## A strip left standing
//!
//! A selection can stop one strip short of the whole network: every blend
//! around the gusset but the one along its sloping top where it meets the wall
//! (the 2026-09-15 "delete face not working for selection" report). What is
//! asked for is then the solid with only THAT edge rounded, and it is as
//! well-posed as the whole network: the kept strip is a cylinder tangent to its
//! two walls, and the strip it mitred into is leaving, so what ends it now is
//! the CAP — the one other plane meeting that junction, the one the deleted
//! strip rounded against the kept strip's wall. The junction no longer shrinks
//! to a point. It splits into TWO vertices on the cap, where each of the kept
//! strip's straight rims runs into it, joined by the cap's section of the
//! cylinder: a circular arc when the cap is perpendicular to the axis, which is
//! the only case read. Every edge arriving at the junction ends on the vertex
//! of the kept wall it lies on; the kept strip is trimmed back to the arc and
//! refit on its own carrier.
//!
//! ## Every face bounded by blends
//!
//! Round all twelve edges of a box and select every blend: twelve strips, eight
//! spherical vertex blends, and six sides each bounded by four rims and nothing
//! else. Nothing new is asked of this lane — every junction is a vertex blend
//! and the three strips ending on it, and three planes pin its vertex — but the
//! patch gate used to claim the set first, because every survivor loop the
//! selection touches is consumed whole, and then refuse it for leaving a face
//! with no loop. It declines that shape now (`encloses_every_survivor` in
//! `delete_faces.rs`), and the set reaches this lane.
//!
//! ## Blend caps
//!
//! Round a box's four vertical edges at r = 5 and then the other eight at
//! r = 3: each r = 3 edge now runs smoothly into an r = 5 arc at both ends, and
//! the second fillet stops its strip there with a planar CAP (`plan_cap_end` in
//! `blend/network.rs`) — a triangle in the strip's section plane bounded by the
//! section arc and two straight legs, one on each wall, meeting at the strip's
//! sharp vertex, its APEX. The fillet names it `{strip}:CAP` (the feature layer
//! adds `[n]`), and that naming is what says whose material it is:
//!
//! * a cap is the END of its strip's fill. Once the strip goes there is
//!   material on both sides of it, so it goes with the strip — picked or not.
//!   The dispatch adds every selected strip's caps before any lane is asked
//!   (`with_owned_caps`), and a cap picked without its strip is refused by name.
//! * in the network a cap is all end edges (its arc on the strip, its legs
//!   across the walls), so it joins the junction its apex sits in. A cap does
//!   not share an edge with the round it ran up to — only the apex — so the
//!   network is one piece when its faces are joined through JUNCTIONS, not
//!   through edges: the two-radius body's 28 blends are twelve edge-connected
//!   pieces and one network.
//! * a junction whose cap apex is HELD — an edge outside the network still ends
//!   on it, the arc and rim of a round that stays — shrinks onto the apex: the
//!   cap's plane and the strip's two walls pin it, and the apex keeps its id.
//!   Where the round goes too, nothing holds the apex, and the corner's own
//!   planes pin the junction as they would without caps.
//! * a cap left STANDING with its own strip, beside a network that is deleted —
//!   the r = 5 rounds selected, the r = 3 strips not — keeps its apex. The
//!   junction the round's end arc shrinks to is the box corner, and each
//!   standing apex in it is joined to that corner by a SPOKE: the straight
//!   edge along the two walls the apex sits on. A plane whose loops wear
//!   neither a network edge nor a moved one — the standing cap — is not
//!   re-trimmed.
//!
//! ## Scope
//!
//! One selection whose faces are joined through their junctions, planar
//! survivors plus kept cylindrical strips and caps, strips with exactly two rim
//! edges. A selection in several pieces, a rimless face bordering a survivor
//! that is not a cap (a pocket's wall), a rim split into several edges, or a
//! selected face tangent to one survivor or to three is not read as a network,
//! and the selection goes on to the component split and the one-at-a-time chain
//! exactly as before. One face is a network only with caps: its own, or caps
//! standing at its ends. A network that DOES read, but has a curved survivor
//! beside it that is not a kept strip of that shape (a sphere, a strip whose cap
//! is oblique to its axis) or that touches it anywhere but a fixed apex, is
//! refused by name: one face at a time has no better answer for a network. A
//! junction holding both a held and a standing apex, or a kept strip and an
//! apex, is refused by name. A single corner blend never gets here: the corner
//! lane takes it first, byte for byte as it always has.

use super::delete_faces::euler_characteristic;
use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// One strip of the network: a selected face with exactly two rims.
pub(super) struct NetworkStrip {
    face_id: u64,
    /// The two rim edges, each with the wall across it.
    rims: [(u64, u64); 2],
    /// The junctions at the start and end of the FIRST rim's edge record.
    junctions: [usize; 2],
}

/// A cylindrical blend strip the selection leaves standing beside the network.
pub(super) struct KeptStrip {
    /// A point on the cylinder's axis, the unit axis, and the radius.
    origin: Vec3,
    axis: Vec3,
    radius: f64,
    /// The carrier's axial extent, measured from `origin` along `axis`.
    axial: [f64; 2],
    /// The two straight rim edges, each with the planar wall across it.
    rims: [(u64, u64); 2],
}

/// A planar end CAP a fillet left where one of its strips runs out onto an
/// edge that continues the blended edge smoothly: a face in the strip's
/// section plane bounded by the strip's section arc and two straight legs, one
/// on each of the strip's walls, meeting at the strip's sharp vertex. It
/// belongs to the network because its strip does (the fillet names it
/// `{strip}:CAP`).
pub(super) struct NetworkCap {
    face_id: u64,
    plane: Plane,
    /// Where the two legs meet: the sharp vertex the strip was cut back from.
    apex: u64,
    /// Whether an edge outside the network still ends on the apex — the
    /// smooth continuation the cap ended the strip against stays, so the apex
    /// stays too, and the cap's plane pins the junction it sits in.
    held: bool,
}

/// A selection the gate read as one planar blend network.
pub(super) struct BlendNetwork {
    shell_index: usize,
    /// Every face the heal removes: the selection, and every cap of a selected
    /// strip.
    pub(super) faces: HashSet<u64>,
    /// Every edge a selected face uses.
    edges: HashSet<u64>,
    /// Every vertex those edges end on.
    vertices: Vec<u64>,
    pub(super) strips: Vec<NetworkStrip>,
    /// Vertex ids per junction, and the junction of every network vertex.
    pub(super) junctions: Vec<Vec<u64>>,
    junction_of: HashMap<u64, usize>,
    /// Rim edge -> the strip it belongs to.
    rim_strip: HashMap<u64, usize>,
    /// Every unselected PLANAR face incident to a network vertex, as its plane.
    planes: HashMap<u64, Plane>,
    /// Every kept strip by face id, and the kept strip ending in each junction
    /// that has one.
    pub(super) kept: HashMap<u64, KeptStrip>,
    kept_at: HashMap<usize, u64>,
    /// Every cap of a strip in the network; each goes with its strip.
    pub(super) caps: Vec<NetworkCap>,
    /// Every cap left beside the network with its own strip, by face id.
    standing_caps: HashMap<u64, StandingCap>,
}

/// What the network gate made of a selection.
pub(super) enum NetworkRead {
    /// Not a blend network; the lanes after this one decide.
    NotANetwork,
    /// A blend network this lane cannot heal, and why.
    Refused(KernelRefusal),
    Network(BlendNetwork),
}

fn union_root(parent: &mut HashMap<u64, u64>, mut node: u64) -> u64 {
    while let Some(&up) = parent.get(&node) {
        if up == node {
            break;
        }
        let grand = parent.get(&up).copied().unwrap_or(up);
        parent.insert(node, grand);
        node = up;
    }
    node
}

fn index_root(parents: &mut [usize], mut index: usize) -> usize {
    while parents[index] != index {
        parents[index] = parents[parents[index]];
        index = parents[index];
    }
    index
}

fn face_title(face: &FaceRecord) -> String {
    match &face.name {
        Some(name) => format!("face {} `{name}`", face.id),
        None => format!("face {}", face.id),
    }
}

/// `(axis point, unit axis, radius)` when the carrier is a circular cylinder:
/// a ruled revolution with equal end radii, or a general revolution whose
/// generatrix is a straight line at one radius (which forces it parallel to the
/// axis — squared radial distance along a line is quadratic, so three equal
/// samples make it constant).
fn cylinder_of(surface: &NurbsSurface, tolerance: f64) -> Option<(Vec3, Vec3, f64)> {
    match surface.analytic()? {
        AnalyticSurface::RuledRevolution {
            frame, rho0, rho1, ..
        } => ((rho0 - rho1).abs() <= tolerance && *rho0 > tolerance)
            .then_some((frame.origin, frame.axis, *rho0)),
        AnalyticSurface::Revolution {
            frame, generatrix, ..
        } => {
            let radial = |point: Vec3| {
                let delta = point.sub(frame.origin);
                delta.sub(frame.axis.scale(delta.dot(frame.axis))).length()
            };
            let [t0, t1] = generatrix.domain().ok()?;
            let start = generatrix.evaluate(t0).ok()?;
            let end = generatrix.evaluate(t1).ok()?;
            let chord = end.sub(start);
            let length = chord.length();
            if length <= tolerance {
                return None;
            }
            let direction = chord.scale(1.0 / length);
            for control in &generatrix.control_points {
                let offset = control.point().ok()?.sub(start);
                if offset.sub(direction.scale(offset.dot(direction))).length() > tolerance {
                    return None;
                }
            }
            let middle = generatrix.evaluate(0.5 * (t0 + t1)).ok()?;
            let rho = radial(start);
            ((radial(end) - rho).abs() <= tolerance
                && (radial(middle) - rho).abs() <= tolerance
                && rho > tolerance)
                .then_some((frame.origin, frame.axis, rho))
        }
        _ => None,
    }
}

/// Read a curved survivor beside the network as a KEPT strip, or say why it is
/// not one.
#[allow(clippy::too_many_arguments)]
fn read_kept_strip(
    face: &FaceRecord,
    planes: &HashMap<u64, Plane>,
    network_edges: &HashSet<u64>,
    network_vertices: &HashSet<u64>,
    owners: &HashMap<u64, Vec<u64>>,
    edge_records: &HashMap<u64, &EdgeRecord>,
    angular: f64,
    plane_tolerance: f64,
) -> Result<KeptStrip, String> {
    if face.loops.len() != 1 {
        return Err(format!("it has {} boundary loops", face.loops.len()));
    }
    let Some((origin, axis, radius)) = cylinder_of(&face.surface, plane_tolerance) else {
        return Err("its carrier is not a cylinder".into());
    };
    let axis = axis.normalized()?;
    if !face.loops[0]
        .coedges
        .iter()
        .any(|coedge| network_edges.contains(&coedge.edge_id))
    {
        return Err("it meets the selection only at a vertex".into());
    }
    let mut rims: Vec<(u64, u64)> = Vec::new();
    for coedge in &face.loops[0].coedges {
        if network_edges.contains(&coedge.edge_id) {
            continue;
        }
        let edge = edge_records
            .get(&coedge.edge_id)
            .ok_or_else(|| format!("its edge {} is missing", coedge.edge_id))?;
        let other = owners
            .get(&coedge.edge_id)
            .and_then(|sharing| sharing.iter().find(|id| **id != face.id))
            .copied()
            .ok_or_else(|| format!("its edge {} has no second face", coedge.edge_id))?;
        let at_network = network_vertices.contains(&edge.start_vertex_id)
            || network_vertices.contains(&edge.end_vertex_id);
        let tangent = match planes.get(&other) {
            Some(plane) => {
                let [q0, q1] = coedge.pcurve.domain()?;
                let uv = coedge.pcurve.evaluate(0.5 * (q0 + q1))?;
                face.surface.normal(uv.x, uv.y)?.cross(plane.normal).length() <= angular
            }
            None => false,
        };
        if tangent {
            // A cylinder tangent to a plane touches it along a ruling.
            let run = edge
                .curve
                .evaluate(edge.t1)?
                .sub(edge.curve.evaluate(edge.t0)?);
            if run.length() <= plane_tolerance || run.normalized()?.cross(axis).length() > angular {
                return Err(format!("its rim edge {} is not a straight ruling", edge.id));
            }
            rims.push((edge.id, other));
        } else if at_network {
            return Err(format!(
                "its edge {} runs into the network without being a rim on a planar wall",
                edge.id
            ));
        }
    }
    if rims.len() != 2 || rims[0].1 == rims[1].1 {
        return Err(format!(
            "it is not a strip between two planar walls ({} tangent rims)",
            rims.len()
        ));
    }
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let mut axial = [f64::INFINITY, f64::NEG_INFINITY];
    for (u, v) in [(u0, v0), (u0, v1), (u1, v0), (u1, v1)] {
        let along = face.surface.evaluate(u, v)?.sub(origin).dot(axis);
        axial = [axial[0].min(along), axial[1].max(along)];
    }
    Ok(KeptStrip {
        origin,
        axis,
        radius,
        axial,
        rims: [rims[0], rims[1]],
    })
}

/// What [`read_cap`] makes of a cap.
struct CapRead {
    strip: u64,
    plane: Plane,
    apex: u64,
    /// The faces across its two legs: the strip's walls.
    walls: [u64; 2],
}

/// A cap left standing beside the network with the strip it ends.
struct StandingCap {
    strip: u64,
    apex: u64,
    walls: [u64; 2],
}

/// Read `face` as a fillet's end cap, or `None` when it is not one.
///
/// Ownership is the fillet's naming — a cap is named `{strip}:CAP` after the
/// strip it ends, and the feature layer's uniqueness pass
/// (`ensure_unique_face_names`) appends `[n]` to the two caps of one strip, and
/// to a strip whose name repeats — and the shape confirms it: a plane with one loop of three
/// edges, one of them shared with that strip (the section arc), and two
/// straight legs meeting at one vertex, each on a face the strip also borders.
fn read_cap(
    solid: &BrepSolid,
    face: &FaceRecord,
    owners: &HashMap<u64, Vec<u64>>,
    edge_records: &HashMap<u64, &EdgeRecord>,
    plane_tolerance: f64,
) -> Option<CapRead> {
    let strip_name = without_index(face.name.as_deref()?).strip_suffix(":CAP")?;
    if face.loops.len() != 1 || face.loops[0].coedges.len() != 3 {
        return None;
    }
    let plane = plane_of_surface(&face.surface, plane_tolerance, "blend cap").ok()?;
    let mut arcs: Vec<(u64, u64)> = Vec::new();
    let mut legs: Vec<(u64, u64)> = Vec::new();
    for coedge in &face.loops[0].coedges {
        let other = across_face(owners, coedge.edge_id, face.id)?;
        let (shell, position) = find_face(solid, other)?;
        let other_name = solid.shells[shell].faces[position].name.as_deref();
        if other_name.is_some_and(|name| name == strip_name || without_index(name) == strip_name) {
            arcs.push((coedge.edge_id, other));
        } else {
            legs.push((coedge.edge_id, other));
        }
    }
    let ([(arc, strip)], [(leg_a, wall_a), (leg_b, wall_b)]) = (arcs.as_slice(), legs.as_slice())
    else {
        return None;
    };
    let (arc, leg_a, leg_b) = (edge_records.get(arc)?, edge_records.get(leg_a)?, edge_records.get(leg_b)?);
    let apex = [leg_a.start_vertex_id, leg_a.end_vertex_id]
        .into_iter()
        .find(|vertex| [leg_b.start_vertex_id, leg_b.end_vertex_id].contains(vertex))?;
    if [arc.start_vertex_id, arc.end_vertex_id].contains(&apex) {
        return None;
    }
    for leg in [leg_a, leg_b] {
        let middle = leg.curve.evaluate(0.5 * (leg.t0 + leg.t1)).ok()?;
        let chord = leg
            .curve
            .evaluate(leg.t0)
            .ok()?
            .add(leg.curve.evaluate(leg.t1).ok()?)
            .scale(0.5);
        if middle.sub(chord).length() > plane_tolerance {
            return None;
        }
    }
    let (shell, position) = find_face(solid, *strip)?;
    let strip_walls: HashSet<u64> = solid.shells[shell].faces[position]
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .filter_map(|coedge| across_face(owners, coedge.edge_id, *strip))
        .collect();
    (wall_a != wall_b && strip_walls.contains(wall_a) && strip_walls.contains(wall_b)).then_some(
        CapRead {
            strip: *strip,
            plane,
            apex,
            walls: [*wall_a, *wall_b],
        },
    )
}

/// `name` without one trailing `[n]` uniqueness index.
fn without_index(name: &str) -> &str {
    name.strip_suffix(']')
        .and_then(|head| head.rsplit_once('['))
        .filter(|(_, index)| !index.is_empty() && index.bytes().all(|byte| byte.is_ascii_digit()))
        .map_or(name, |(base, _)| base)
}

/// The other face using `edge_id`, seen from `face_id`.
fn across_face(owners: &HashMap<u64, Vec<u64>>, edge_id: u64, face_id: u64) -> Option<u64> {
    owners.get(&edge_id)?.iter().copied().find(|id| *id != face_id)
}

/// The caps a selection touches: the ones its strips own, and the ones left
/// standing with their own strips.
struct CapScan {
    owned: Vec<(u64, CapRead)>,
    standing: HashMap<u64, StandingCap>,
}

/// Read every cap in the shell against `selected`. A cap is blend material: it
/// bounds its fill's end, and once the strip is gone there is material on both
/// sides of it. So a selected strip takes its caps whether or not they were
/// picked, and a cap picked without its strip is refused — it is not a question
/// this lane, or one face at a time, can answer.
fn scan_caps(
    solid: &BrepSolid,
    selected: &HashSet<u64>,
    owners: &HashMap<u64, Vec<u64>>,
    edge_records: &HashMap<u64, &EdgeRecord>,
    plane_tolerance: f64,
    op: &str,
) -> Result<CapScan, KernelRefusal> {
    let mut scan = CapScan {
        owned: Vec::new(),
        standing: HashMap::default(),
    };
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(cap) = read_cap(solid, face, owners, edge_records, plane_tolerance) else {
            continue;
        };
        if selected.contains(&cap.strip) {
            scan.owned.push((face.id, cap));
        } else if selected.contains(&face.id) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Collect,
                "cap_without_strip",
                format!(
                    "{op}: {} is the end cap of blend face {}, which is not selected — a cap is the \
                 end of its strip's fill and goes with it; select the strip as well",
                    face_title(face),
                    cap.strip
                ),
            ));
        } else {
            scan.standing.insert(
                face.id,
                StandingCap {
                    strip: cap.strip,
                    apex: cap.apex,
                    walls: cap.walls,
                },
            );
        }
    }
    Ok(scan)
}

/// `face_ids` with every cap its strips own appended (see [`scan_caps`]), so
/// that every lane is asked about a strip together with its caps.
pub(super) fn with_owned_caps(solid: &BrepSolid, face_ids: &[u64], op: &str) -> Result<Vec<u64>, KernelRefusal> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let edge_records: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            owners.entry(coedge.edge_id).or_default().push(face.id);
        }
    }
    let plane_tolerance = (solid_model_scale(solid) * 1e-6).max(1e-7);
    let scan = scan_caps(solid, &selected, &owners, &edge_records, plane_tolerance, op)?;
    let mut with_caps = face_ids.to_vec();
    with_caps.extend(
        scan.owned
            .iter()
            .map(|(face_id, _)| *face_id)
            .filter(|face_id| !selected.contains(face_id)),
    );
    Ok(with_caps)
}


/// The network gate with its refusal: [`NetworkRead::Refused`] names a curved
/// survivor beside a selection that otherwise reads as a network.
pub(super) fn read_blend_network(solid: &BrepSolid, face_ids: &[u64], op: &str) -> NetworkRead {
    read_network(solid, face_ids, op).unwrap_or(NetworkRead::NotANetwork)
}

fn read_network(solid: &BrepSolid, face_ids: &[u64], op: &str) -> Option<NetworkRead> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    if selected.is_empty() {
        return None;
    }
    let shell_index = find_face(solid, face_ids[0])?.0;
    for face_id in face_ids {
        if find_face(solid, *face_id)?.0 != shell_index {
            return None;
        }
    }
    let scale = solid_model_scale(solid);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let angular = crate::KernelTolerances::for_solid(solid, 1e-7).angular;

    let edge_records: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    // Every face using each edge.
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            owners.entry(coedge.edge_id).or_default().push(face.id);
        }
    }

    // --- Caps go with the strips they end -------------------------------------
    let scan = match scan_caps(solid, &selected, &owners, &edge_records, plane_tolerance, op) {
        Ok(scan) => scan,
        Err(reason) => return Some(NetworkRead::Refused(reason)),
    };
    let mut faces = selected.clone();
    faces.extend(scan.owned.iter().map(|(face_id, _)| *face_id));
    let standing_caps = scan.standing;
    let mut ordered: Vec<u64> = faces.iter().copied().collect();
    ordered.sort_unstable();

    let mut network_edges: HashSet<u64> = HashSet::default();
    for face_id in &ordered {
        let (shell, position) = find_face(solid, *face_id)?;
        for coedge in solid.shells[shell].faces[position]
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            network_edges.insert(coedge.edge_id);
        }
    }
    let mut vertex_set: HashSet<u64> = HashSet::default();
    for edge_id in &network_edges {
        let edge = edge_records.get(edge_id)?;
        if edge.degenerate || owners.get(edge_id).map_or(0, Vec::len) != 2 {
            return None;
        }
        vertex_set.insert(edge.start_vertex_id);
        vertex_set.insert(edge.end_vertex_id);
    }
    let caps: Vec<NetworkCap> = scan
        .owned
        .into_iter()
        .map(|(face_id, CapRead { plane, apex, .. })| NetworkCap {
            face_id,
            plane,
            apex,
            held: solid.edges.iter().any(|edge| {
                !network_edges.contains(&edge.id)
                    && (edge.start_vertex_id == apex || edge.end_vertex_id == apex)
            }),
        })
        .collect();
    // One face is a network only with the caps it takes or the caps standing at
    // its ends: a lone strip with neither is the one-face chain's question.
    if faces.len() < 2 && !standing_caps.values().any(|cap| vertex_set.contains(&cap.apex)) {
        return None;
    }
    // The apexes the heal leaves where they are: a held apex, and the apex of a
    // cap that stays with its strip.
    let fixed_apexes: HashSet<u64> = caps
        .iter()
        .filter(|cap| cap.held)
        .map(|cap| cap.apex)
        .chain(standing_caps.values().map(|cap| cap.apex))
        .filter(|apex| vertex_set.contains(apex))
        .collect();

    // Every survivor that meets the network at a vertex is a plane or a curved
    // face; a curved one has to be a kept strip, or the network is refused
    // once the rest of it reads — unless all it touches is a held cap apex,
    // which the heal leaves where it is.
    let mut planes: HashMap<u64, Plane> = HashMap::default();
    let mut curved: Vec<u64> = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if faces.contains(&face.id) {
            continue;
        }
        let edges = || {
            face.loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter_map(|coedge| edge_records.get(&coedge.edge_id))
        };
        let touched: HashSet<u64> = edges()
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .filter(|vertex| vertex_set.contains(vertex))
            .collect();
        if touched.is_empty() {
            continue;
        }
        if find_face(solid, face.id)?.0 != shell_index {
            return None;
        }
        match plane_of_surface(&face.surface, plane_tolerance, "blend network") {
            Ok(plane) => {
                planes.insert(face.id, plane);
            }
            Err(_) => {
                let at_apexes_only = touched.is_subset(&fixed_apexes)
                    && !edges().any(|edge| network_edges.contains(&edge.id));
                if !at_apexes_only {
                    curved.push(face.id);
                }
            }
        }
    }
    curved.sort_unstable();
    let mut kept: HashMap<u64, KeptStrip> = HashMap::default();
    let mut refusals: Vec<KernelRefusal> = Vec::new();
    for face_id in &curved {
        let (shell, position) = find_face(solid, *face_id)?;
        let face = &solid.shells[shell].faces[position];
        match read_kept_strip(
            face,
            &planes,
            &network_edges,
            &vertex_set,
            &owners,
            &edge_records,
            angular,
            plane_tolerance,
        ) {
            Ok(strip) => {
                kept.insert(*face_id, strip);
            }
            Err(reason) => refusals.push(KernelRefusal::unsupported(
                KernelStage::Classify,
                "kept_face",
                format!(
                    "{op}: {} is left beside the selected blend network but cannot stay: {reason} \
                 — only planar faces and straight cylindrical blend strips between two planar \
                 walls can be kept beside a deleted network; select it as well",
                    face_title(face)
                ),
            )),
        }
    }

    // --- Rims and end edges ---------------------------------------------------
    let mut end_edges: HashSet<u64> = HashSet::default();
    let mut strips: Vec<NetworkStrip> = Vec::new();
    let mut rim_strip: HashMap<u64, usize> = HashMap::default();
    let cap_faces: HashSet<u64> = caps.iter().map(|cap| cap.face_id).collect();
    for &face_id in &ordered {
        let (shell, position) = find_face(solid, face_id)?;
        let face = &solid.shells[shell].faces[position];
        let mut rims: Vec<(u64, u64)> = Vec::new();
        let mut meets_survivor = false;
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let other = *owners[&coedge.edge_id].iter().find(|id| **id != face_id)?;
            // A kept strip is where a deleted strip ENDS: a mitre, never a rim,
            // even where the two are tangent (a vertex blend is tangent to all
            // three of its strips).
            if faces.contains(&other) || kept.contains_key(&other) {
                end_edges.insert(coedge.edge_id);
                continue;
            }
            meets_survivor = true;
            // Tangent across the edge's midpoint: the blend's own normal from
            // its pcurve, against the survivor's plane — or, for a curved
            // survivor, its normal at the same point.
            let [q0, q1] = coedge.pcurve.domain().ok()?;
            let uv = coedge.pcurve.evaluate(0.5 * (q0 + q1)).ok()?;
            let normal = face.surface.normal(uv.x, uv.y).ok()?;
            let across = match planes.get(&other) {
                Some(plane) => plane.normal,
                None => {
                    let (other_shell, other_position) = find_face(solid, other)?;
                    let surface = &solid.shells[other_shell].faces[other_position].surface;
                    let point = face.surface.evaluate(uv.x, uv.y).ok()?;
                    let foot = crate::project_point_to_surface(surface, point).ok()?;
                    surface.normal(foot.u, foot.v).ok()?
                }
            };
            if normal.cross(across).length() <= angular {
                rims.push((coedge.edge_id, other));
            } else {
                end_edges.insert(coedge.edge_id);
            }
        }
        match rims.len() {
            // A vertex blend is enclosed by the strips that meet on it, and a
            // cap is all end edges: its arc on the strip, its legs across the
            // walls. Any other face with no rim that borders a survivor — a
            // pocket's wall or floor — is not a blend at all.
            0 if !meets_survivor || cap_faces.contains(&face_id) => {}
            2 if rims[0].1 != rims[1].1 && rims[0].0 != rims[1].0 => {
                for (edge_id, _) in &rims {
                    rim_strip.insert(*edge_id, strips.len());
                }
                strips.push(NetworkStrip {
                    face_id,
                    rims: [rims[0], rims[1]],
                    junctions: [0, 0],
                });
            }
            _ => return None,
        }
    }
    if strips.is_empty() {
        return None;
    }

    // --- Junctions: clusters of end edges -------------------------------------
    let mut parent: HashMap<u64, u64> = HashMap::default();
    let mut sorted_ends: Vec<u64> = end_edges.iter().copied().collect();
    sorted_ends.sort_unstable();
    for edge_id in &sorted_ends {
        let edge = edge_records[edge_id];
        parent.entry(edge.start_vertex_id).or_insert(edge.start_vertex_id);
        parent.entry(edge.end_vertex_id).or_insert(edge.end_vertex_id);
        let a = union_root(&mut parent, edge.start_vertex_id);
        let b = union_root(&mut parent, edge.end_vertex_id);
        if a != b {
            parent.insert(a.max(b), a.min(b));
        }
    }
    let mut vertices: Vec<u64> = vertex_set.iter().copied().collect();
    vertices.sort_unstable();
    let mut root_junction: HashMap<u64, usize> = HashMap::default();
    let mut junctions: Vec<Vec<u64>> = Vec::new();
    let mut junction_of: HashMap<u64, usize> = HashMap::default();
    for &vertex_id in &vertices {
        // A network vertex no end edge reaches would be a rim running straight
        // into another rim — not a blend network.
        if !parent.contains_key(&vertex_id) {
            return None;
        }
        let root = union_root(&mut parent, vertex_id);
        let index = *root_junction.entry(root).or_insert_with(|| {
            junctions.push(Vec::new());
            junctions.len() - 1
        });
        junctions[index].push(vertex_id);
        junction_of.insert(vertex_id, index);
    }

    // --- One piece, joined through its junctions ------------------------------
    // Faces that share an edge share its junctions, and so do faces that meet
    // only at a vertex a junction shrinks — the rounds a first fillet left and
    // the capped strips a second one ran up to them. A selection in several
    // such pieces goes to the component split, where each piece — a pocket
    // that caps, a lone strip the chain takes — is asked its own question.
    let mut junction_root: Vec<usize> = (0..junctions.len()).collect();
    let mut face_junction: Vec<usize> = Vec::with_capacity(ordered.len());
    for face_id in &ordered {
        let (shell, position) = find_face(solid, *face_id)?;
        let mut touched = solid.shells[shell].faces[position]
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .flat_map(|coedge| {
                let edge = edge_records[&coedge.edge_id];
                [edge.start_vertex_id, edge.end_vertex_id]
            })
            .map(|vertex| junction_of[&vertex]);
        let first = touched.next()?;
        for other in touched {
            let (a, b) = (index_root(&mut junction_root, first), index_root(&mut junction_root, other));
            junction_root[a.max(b)] = a.min(b);
        }
        face_junction.push(first);
    }
    let piece = index_root(&mut junction_root, face_junction[0]);
    if face_junction
        .iter()
        .any(|junction| index_root(&mut junction_root, *junction) != piece)
    {
        return None;
    }

    // --- Every strip runs between two distinct junctions -----------------------
    for strip in &mut strips {
        let ends = |edge_id: u64| -> (usize, usize) {
            let edge = edge_records[&edge_id];
            (junction_of[&edge.start_vertex_id], junction_of[&edge.end_vertex_id])
        };
        let (a0, b0) = ends(strip.rims[0].0);
        let (a1, b1) = ends(strip.rims[1].0);
        if a0 == b0 || !((a0 == a1 && b0 == b1) || (a0 == b1 && b0 == a1)) {
            return None;
        }
        strip.junctions = [a0, b0];
    }

    // --- Every kept strip ends in its own junctions, across its whole width ----
    let mut kept_at: HashMap<usize, u64> = HashMap::default();
    let mut kept_ids: Vec<u64> = kept.keys().copied().collect();
    kept_ids.sort_unstable();
    for kept_id in kept_ids {
        let (shell, position) = find_face(solid, kept_id)?;
        let face = &solid.shells[shell].faces[position];
        let mut touched: Vec<usize> = face.loops[0]
            .coedges
            .iter()
            .filter(|coedge| network_edges.contains(&coedge.edge_id))
            .flat_map(|coedge| {
                let edge = edge_records[&coedge.edge_id];
                [junction_of[&edge.start_vertex_id], junction_of[&edge.end_vertex_id]]
            })
            .collect();
        touched.sort_unstable();
        touched.dedup();
        for junction in touched {
            let whole_width = kept[&kept_id].rims.iter().all(|(rim, _)| {
                let edge = edge_records[rim];
                [edge.start_vertex_id, edge.end_vertex_id]
                    .iter()
                    .filter(|vertex| junction_of.get(vertex) == Some(&junction))
                    .count()
                    == 1
            });
            if !whole_width {
                refusals.push(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "kept_strip_end",
                    format!(
                        "{op}: {} is left beside the selected blend network but does not end in \
                     it across its whole width — refusing rather than guessing its end",
                        face_title(face)
                    ),
                ));
            }
            if let Some(other) = kept_at.insert(junction, kept_id) {
                refusals.push(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "two_kept_at_junction",
                    format!(
                        "{op}: blend faces {other} and {kept_id} are both left standing where they \
                     end in one junction of the selected blend network — select one of them as well"
                    ),
                ));
            }
        }
    }
    if let Some(reason) = refusals.into_iter().next() {
        return Some(NetworkRead::Refused(reason));
    }

    Some(NetworkRead::Network(BlendNetwork {
        shell_index,
        faces,
        edges: network_edges,
        vertices,
        strips,
        junctions,
        junction_of,
        rim_strip,
        planes,
        kept,
        kept_at,
        caps,
        standing_caps,
    }))
}

/// The unique point three planes share, or `None` when they do not pin one.
fn three_plane_point(a: &Plane, b: &Plane, c: &Plane) -> Option<(Vec3, f64)> {
    let determinant = a.normal.dot(b.normal.cross(c.normal));
    if determinant.abs() <= PARALLEL_EPS {
        return None;
    }
    let da = a.normal.dot(a.origin);
    let db = b.normal.dot(b.origin);
    let dc = c.normal.dot(c.origin);
    let point = b
        .normal
        .cross(c.normal)
        .scale(da)
        .add(c.normal.cross(a.normal).scale(db))
        .add(a.normal.cross(b.normal).scale(dc))
        .scale(1.0 / determinant);
    Some((point, determinant.abs()))
}

/// What one junction recovers: the unblended vertex, or — where a kept strip
/// ends — one vertex per kept wall, joined by the cap's section arc.
enum JunctionEnd {
    Point {
        vertex: u64,
        point: Vec3,
    },
    Split {
        kept: u64,
        walls: [u64; 2],
        vertices: [u64; 2],
        points: [Vec3; 2],
    },
}

impl JunctionEnd {
    /// The vertex an edge flanked by `flanking` ends on at this junction.
    fn end_for(&self, flanking: &[u64], what: &str, op: &str) -> Result<(u64, Vec3), KernelRefusal> {
        match self {
            JunctionEnd::Point { vertex, point } => Ok((*vertex, *point)),
            JunctionEnd::Split {
                kept,
                walls,
                vertices,
                points,
            } => {
                let sides: Vec<usize> = (0..2).filter(|side| flanking.contains(&walls[*side])).collect();
                match sides.as_slice() {
                    [side] => Ok((vertices[*side], points[*side])),
                    _ => Err(KernelRefusal::ill_posed(
                        KernelStage::Sew,
                        "kept_end_side",
                        format!(
                            "{op}: {what} reaches the end of kept blend face {kept} on {} of its \
                         walls — refusing rather than guessing which rim it ends on",
                            if sides.is_empty() { "neither" } else { "both" }
                        ),
                    )),
                }
            }
        }
    }
}

/// Heal a blend network: every junction shrinks to the vertex its planes meet
/// in (or splits across a kept strip's end), every strip becomes the sharp edge
/// between its two junctions, and the network goes.
pub(super) fn heal_blend_network(
    solid: &BrepSolid,
    network: &BlendNetwork,
    op: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let mut solid = solid.clone();
    let chi_before = euler_characteristic(&solid);
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let angular = crate::KernelTolerances::for_solid(&solid, 1e-7).angular;

    let edge_records: HashMap<u64, EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            owners.entry(coedge.edge_id).or_default().push(face.id);
        }
    }

    // --- The network's own region gate (centre + reach) ---------------------
    let mut centre = Vec3::default();
    for &vertex_id in &network.vertices {
        centre = centre.add(edge_point(&solid, vertex_id)?);
    }
    centre = centre.scale(1.0 / network.vertices.len() as f64);
    let mut reach = 0.0f64;
    for &vertex_id in &network.vertices {
        reach = reach.max(edge_point(&solid, vertex_id)?.sub(centre).length());
    }
    let reach = reach * 3.0 + tolerance;
    let outside_region = |point: Vec3| point.sub(centre).length() > reach;

    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };

    // --- Each junction's vertex: where every plane meeting there agrees -----
    let mut ends: Vec<JunctionEnd> = Vec::with_capacity(network.junctions.len());
    let mut arcs: Vec<EdgeRecord> = Vec::new();
    // Straight edges from a standing cap's apex to its junction's vertex, each
    // with the two walls it lies on.
    let mut spokes: Vec<(EdgeRecord, [u64; 2])> = Vec::new();
    for (junction_index, junction) in network.junctions.iter().enumerate() {
        let members: HashSet<u64> = junction.iter().copied().collect();
        let mut incident: Vec<u64> = network
            .planes
            .keys()
            .copied()
            .filter(|face_id| {
                let Some((shell, position)) = find_face(&solid, *face_id) else {
                    return false;
                };
                solid.shells[shell].faces[position]
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .filter_map(|coedge| edge_records.get(&coedge.edge_id))
                    .any(|edge| {
                        members.contains(&edge.start_vertex_id)
                            || members.contains(&edge.end_vertex_id)
                    })
            })
            .collect();
        incident.sort_unstable();
        let near = edge_point(&solid, junction[0])?;

        let held: Vec<&NetworkCap> = network
            .caps
            .iter()
            .filter(|cap| cap.held && members.contains(&cap.apex))
            .collect();
        let mut standing: Vec<(u64, &StandingCap)> = network
            .standing_caps
            .iter()
            .filter(|(_, cap)| members.contains(&cap.apex))
            .map(|(face_id, cap)| (*face_id, cap))
            .collect();
        standing.sort_unstable_by_key(|(face_id, _)| *face_id);
        if let (Some(cap), Some((standing_id, _))) = (held.first(), standing.first()) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "held_and_standing",
                format!(
                    "{op}: the blend junction near ({:.6}, {:.6}, {:.6}) holds the apex of blend cap \
                 face {} and the apex of blend cap face {standing_id}, which stays — reading \
                 both in one junction is not supported",
                    near.x, near.y, near.z, cap.face_id
                ),
            ));
        }

        if let Some(kept_id) = network.kept_at.get(&junction_index) {
            if let Some(cap) = held
                .first()
                .map(|cap| cap.face_id)
                .or(standing.first().map(|(face_id, _)| *face_id))
            {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "kept_and_apex",
                    format!(
                        "{op}: kept blend face {kept_id} ends in the blend junction near \
                     ({:.6}, {:.6}, {:.6}), which also holds the apex of blend cap face {} — \
                     splitting a junction across both is not supported",
                        near.x, near.y, near.z, cap
                    ),
                ));
            }
            let kept = &network.kept[kept_id];
            let walls = [kept.rims[0].1, kept.rims[1].1];
            let caps: Vec<u64> = incident
                .iter()
                .copied()
                .filter(|face_id| !walls.contains(face_id))
                .collect();
            if caps.len() != 1 || !walls.iter().all(|wall| incident.contains(wall)) {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "kept_cap_count",
                    format!(
                        "{op}: kept blend face {kept_id} ends in the blend junction near \
                     ({:.6}, {:.6}, {:.6}) against {} faces besides its walls — one cap plane \
                     is needed to end it",
                        near.x,
                        near.y,
                        near.z,
                        caps.len()
                    ),
                ));
            }
            let cap = &network.planes[&caps[0]];
            if cap.normal.cross(kept.axis).length() > angular {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "oblique_cap",
                    format!(
                        "{op}: face {} would end kept blend face {kept_id} obliquely to its axis — \
                     that section is an ellipse, which this lane does not build",
                        caps[0]
                    ),
                ));
            }
            let mut points = [Vec3::default(); 2];
            for side in 0..2 {
                let rim = &edge_records[&kept.rims[side].0];
                let on_rim = rim
                    .curve
                    .evaluate(0.5 * (rim.t0 + rim.t1))
                    .or_refuse(KernelStage::Sew, "evaluate")?;
                let wall = &network.planes[&walls[side]];
                let point = intersect_line_plane(
                    &Line {
                        point: on_rim,
                        dir: kept.axis,
                    },
                    cap,
                )
                .ok_or_else(|| {
                    KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "rim_parallel_cap",
                        format!("{op}: kept blend face {kept_id}'s rim runs parallel to its cap"),
                    )
                })?;
                let delta = point.sub(kept.origin);
                let along = delta.dot(kept.axis);
                let radial = delta.sub(kept.axis.scale(along)).length();
                if point.sub(wall.origin).dot(wall.normal).abs() > plane_tolerance
                    || (radial - kept.radius).abs() > plane_tolerance
                {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "kept_rim_cap_miss",
                        format!(
                            "{op}: kept blend face {kept_id}'s rim on face {} does not meet its cap \
                         face {} on both the wall and the cylinder",
                            walls[side], caps[0]
                        ),
                    ));
                }
                if along < kept.axial[0] - plane_tolerance || along > kept.axial[1] + plane_tolerance
                {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "kept_carrier_short",
                        format!(
                            "{op}: kept blend face {kept_id} does not reach its cap face {} — \
                         extending its carrier is not supported",
                            caps[0]
                        ),
                    ));
                }
                if outside_region(point) {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "kept_end_outside",
                        format!(
                            "{op}: the end of kept blend face {kept_id} lands outside the network's \
                         own region"
                        ),
                    ));
                }
                points[side] = point;
            }

            // The cap's section of the cylinder, from the first wall's rim to
            // the second's, the short way: a blend between two planes sweeps
            // less than half a turn.
            let centre_point = intersect_line_plane(
                &Line {
                    point: kept.origin,
                    dir: kept.axis,
                },
                cap,
            )
            .ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "axis_misses_cap",
                    format!("{op}: kept blend face {kept_id}'s axis misses its cap"),
                )
            })?;
            let x_axis = points[0]
                .sub(centre_point)
                .normalized()
                .or_refuse(KernelStage::Sew, "section_axis")?;
            let to_second = points[1].sub(centre_point);
            let across = to_second.sub(x_axis.scale(to_second.dot(x_axis)));
            if across.length() <= tolerance {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "degenerate_section",
                    format!("{op}: kept blend face {kept_id}'s rims meet its cap at a degenerate section"),
                ));
            }
            let y_axis = across.normalized().or_refuse(KernelStage::Sew, "section_axis")?;
            let sweep = to_second.dot(y_axis).atan2(to_second.dot(x_axis));
            // The strip itself must lie on that short arc: read the azimuth of
            // the mitre it wore at this junction.
            let (kept_shell, kept_position) = find_face(&solid, *kept_id).ok_or_else(|| {
                KernelRefusal::internal(KernelStage::Collect, "face", format!("{op}: missing face {kept_id}"))
            })?;
            let mitre = solid.shells[kept_shell].faces[kept_position]
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter_map(|coedge| edge_records.get(&coedge.edge_id))
                .find(|edge| {
                    network.edges.contains(&edge.id)
                        && members.contains(&edge.start_vertex_id)
                        && members.contains(&edge.end_vertex_id)
                })
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Sew,
                        "kept_end_edge",
                        format!("{op}: kept blend face {kept_id} wears no end edge at its junction"),
                    )
                })?;
            let inside = mitre
                .curve
                .evaluate(0.5 * (mitre.t0 + mitre.t1))
                .or_refuse(KernelStage::Sew, "evaluate")?
                .sub(kept.origin);
            let azimuth = inside.dot(y_axis).atan2(inside.dot(x_axis));
            if !(azimuth > 0.0 && azimuth < sweep) {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "long_arc",
                    format!(
                        "{op}: kept blend face {kept_id} does not lie on the short arc between its \
                     rims at its cap face {}",
                        caps[0]
                    ),
                ));
            }
            let vertices = [alloc(), alloc()];
            let curve = crate::make_arc(centre_point, x_axis, y_axis, kept.radius, 0.0, sweep)
                .or_refuse(KernelStage::Sew, "make_arc")?;
            let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            let arc_at = |t: f64| curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate");
            if arc_at(d0)?.sub(points[0]).length() > plane_tolerance
                || arc_at(d1)?.sub(points[1]).length() > plane_tolerance
            {
                return Err(KernelRefusal::internal(
                    KernelStage::Sew,
                    "arc_ends",
                    format!("{op}: the section arc ending kept blend face {kept_id} misses its rims"),
                ));
            }
            arcs.push(EdgeRecord {
                id: alloc(),
                curve,
                t0: d0,
                t1: d1,
                start_vertex_id: vertices[0],
                end_vertex_id: vertices[1],
                degenerate: false,
                name: None,
            });
            ends.push(JunctionEnd::Split {
                kept: *kept_id,
                walls,
                vertices,
                points,
            });
            continue;
        }

        // The best-conditioned triple pins the vertex; every plane must hold it.
        // A cap whose apex is held stands in for the continuation it ended its
        // strip against: that edge stays, so the junction shrinks onto the apex,
        // and the cap's plane is the one that says where along the walls.
        //
        // A cap that stays is the end of ITS strip's fill, not a face this
        // junction shrinks onto: its apex keeps its place, and a spoke joins it
        // to the junction's vertex along the two walls the apex sits on.
        let pinning: Vec<(u64, &Plane)> = incident
            .iter()
            .filter(|face_id| !standing.iter().any(|(standing_id, _)| standing_id == *face_id))
            .map(|face_id| (*face_id, &network.planes[face_id]))
            .chain(held.iter().map(|cap| (cap.face_id, &cap.plane)))
            .collect();
        let mut best: Option<(Vec3, f64)> = None;
        for i in 0..pinning.len() {
            for j in i + 1..pinning.len() {
                for k in j + 1..pinning.len() {
                    if let Some(found) = three_plane_point(pinning[i].1, pinning[j].1, pinning[k].1) {
                        if best.map_or(true, |(_, conditioning)| found.1 > conditioning) {
                            best = Some(found);
                        }
                    }
                }
            }
        }
        let Some((point, _)) = best else {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "junction_unpinned",
                format!(
                    "{op}: the {} faces meeting at the blend junction near ({:.6}, {:.6}, {:.6}) \
                 do not pin a vertex — refusing rather than emitting an invalid solid",
                    pinning.len(),
                    near.x,
                    near.y,
                    near.z
                ),
            ));
        };
        for (face_id, plane) in &pinning {
            let miss = point.sub(plane.origin).dot(plane.normal).abs();
            if miss > plane_tolerance {
                let what = match network.standing_caps.get(face_id) {
                    Some(StandingCap { strip, .. }) => format!(
                        "blend cap face {face_id} stays with blend face {strip}, which is not \
                         selected, and it"
                    ),
                    None => format!("face {face_id}"),
                };
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "junction_disagree",
                    format!(
                        "{op}: {what} misses the vertex its blend junction near \
                     ({:.6}, {:.6}, {:.6}) recovers by {miss:.6} — the faces there do not \
                     meet in one point",
                        near.x, near.y, near.z
                    ),
                ));
            }
        }
        if outside_region(point) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "junction_outside",
                format!(
                    "{op}: the blend junction near ({:.6}, {:.6}, {:.6}) recovers a vertex outside \
                 the network's own region",
                    near.x, near.y, near.z
                ),
            ));
        }
        let vertex = match held.as_slice() {
            [] => alloc(),
            [cap, rest @ ..] => {
                let apex = edge_point(&solid, cap.apex)?;
                if rest.iter().any(|other| other.apex != cap.apex)
                    || apex.sub(point).length() > plane_tolerance
                {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "held_apex_moves",
                        format!(
                            "{op}: the blend junction near ({:.6}, {:.6}, {:.6}) recovers \
                         ({:.6}, {:.6}, {:.6}), which is not the apex of its blend cap face {} — \
                         the edge that continues past the cap stays, so its vertex cannot move",
                            near.x, near.y, near.z, point.x, point.y, point.z, cap.face_id
                        ),
                    ));
                }
                cap.apex
            }
        };
        for (cap_id, cap) in &standing {
            let apex = edge_point(&solid, cap.apex)?;
            let on_walls = cap.walls.iter().all(|wall| {
                network.planes.get(wall).is_some_and(|plane| {
                    [apex, point]
                        .iter()
                        .all(|at| at.sub(plane.origin).dot(plane.normal).abs() <= plane_tolerance)
                })
            });
            if !on_walls || apex.sub(point).length() <= tolerance {
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "standing_apex_join",
                    format!(
                        "{op}: blend cap face {cap_id} stays with blend face {}, but the blend \
                     junction near ({:.6}, {:.6}, {:.6}) recovers ({:.6}, {:.6}, {:.6}), which is \
                     {} — refusing rather than guessing how the cap's apex joins it",
                        cap.strip,
                        near.x,
                        near.y,
                        near.z,
                        point.x,
                        point.y,
                        point.z,
                        if on_walls { "the apex itself" } else { "not on both of the cap's walls" }
                    ),
                ));
            }
            spokes.push((
                EdgeRecord {
                    id: alloc(),
                    curve: make_line(apex, point).or_refuse(KernelStage::Sew, "make_line")?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: cap.apex,
                    end_vertex_id: vertex,
                    degenerate: false,
                    name: None,
                },
                cap.walls,
            ));
        }
        ends.push(JunctionEnd::Point { vertex, point });
    }

    // --- One sharp edge per strip --------------------------------------------
    let mut sharp_edges: Vec<EdgeRecord> = Vec::with_capacity(network.strips.len());
    let mut spans: HashSet<(u64, u64, u64, u64)> = HashSet::default();
    for strip in &network.strips {
        let [junction_a, junction_b] = strip.junctions;
        let [wall_a, wall_b] = [strip.rims[0].1, strip.rims[1].1];
        let what = format!("the sharp edge recovered for blend face {}", strip.face_id);
        let (start_vertex, start) = ends[junction_a].end_for(&[wall_a, wall_b], &what, op)?;
        let (end_vertex, end) = ends[junction_b].end_for(&[wall_a, wall_b], &what, op)?;
        if start.sub(end).length() <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "edge_collapse",
                format!(
                    "{op}: the sharp edge recovered for blend face {} collapses to a point",
                    strip.face_id
                ),
            ));
        }
        if intersect_planes(&network.planes[&wall_a], &network.planes[&wall_b]).is_none() {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "parallel_walls",
                format!(
                    "{op}: the walls flanking blend face {} are parallel — they cannot \
                 re-intersect into a sharp edge",
                    strip.face_id
                ),
            ));
        }
        let key = (
            junction_a.min(junction_b) as u64,
            junction_a.max(junction_b) as u64,
            wall_a.min(wall_b),
            wall_a.max(wall_b),
        );
        if !spans.insert(key) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "duplicate_edge",
                format!(
                    "{op}: two blend strips recover the same sharp edge (blend face {})",
                    strip.face_id
                ),
            ));
        }
        sharp_edges.push(EdgeRecord {
            id: alloc(),
            curve: make_line(start, end).or_refuse(KernelStage::Sew, "make_line")?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: start_vertex,
            end_vertex_id: end_vertex,
            degenerate: false,
            name: None,
        });
    }

    // --- Relocate every surviving edge that ended on a network vertex --------
    let fixed_apexes: HashSet<u64> = network
        .caps
        .iter()
        .filter(|cap| cap.held)
        .map(|cap| cap.apex)
        .chain(spokes.iter().map(|(spoke, _)| spoke.start_vertex_id))
        .collect();
    let mut pending: Vec<(usize, Option<(u64, Vec3)>, Option<(u64, Vec3)>)> = Vec::new();
    for (index, edge) in solid.edges.iter().enumerate() {
        if network.edges.contains(&edge.id) {
            continue;
        }
        let at_start = network.junction_of.get(&edge.start_vertex_id);
        let at_end = network.junction_of.get(&edge.end_vertex_id);
        if at_start.is_none() && at_end.is_none() {
            continue;
        }
        let flanking = owners.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
        let what = format!("edge {}", edge.id);
        // An end on a held apex, or on a standing cap's apex, does not move.
        let target = |junction: Option<&usize>, vertex: u64| -> Result<Option<(u64, Vec3)>, KernelRefusal> {
            if fixed_apexes.contains(&vertex) {
                return Ok(None);
            }
            Ok(match junction {
                Some(junction) => Some(ends[*junction].end_for(flanking, &what, op)?)
                    .filter(|(target, _)| *target != vertex),
                None => None,
            })
        };
        let start_target = target(at_start, edge.start_vertex_id)?;
        let end_target = target(at_end, edge.end_vertex_id)?;
        if start_target.is_none() && end_target.is_none() {
            continue;
        }
        pending.push((index, start_target, end_target));
    }
    let relocated: HashSet<u64> = pending
        .iter()
        .map(|(index, _, _)| solid.edges[*index].id)
        .collect();
    let unmoved = solid.edges.clone();
    for (index, start_target, end_target) in pending {
        let mut record = solid.edges[index].clone();
        let widened = relocate_open_side_edge(
            &mut record,
            start_target,
            end_target,
            plane_tolerance,
            tolerance,
            op,
        )?;
        if !widened {
            match rederive_side_edge_on_planes(
                &solid,
                &solid.edges[index],
                &network.planes,
                start_target,
                end_target,
                plane_tolerance,
                tolerance,
            )? {
                Some(rebuilt) => record = rebuilt,
                None => {
                    record = rederive_side_edge_on_carriers(
                        &solid,
                        &solid.edges[index],
                        start_target,
                        end_target,
                        plane_tolerance,
                        tolerance,
                        op,
                    )?
                }
            }
        }
        solid.edges[index] = record;
    }
    moved_edges_lie_on_their_faces(&unmoved, &solid, &relocated, &network.planes, plane_tolerance, op)?;

    // --- Survivors: rims become sharp edges, end edges go ---------------------
    let mut orientation: HashMap<u64, Vec<bool>> = HashMap::default();
    let mut touched_faces: Vec<u64> = Vec::new();
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            if network.faces.contains(&face.id) {
                continue;
            }
            let uses_network = face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .any(|coedge| network.edges.contains(&coedge.edge_id));
            if !uses_network {
                continue;
            }
            touched_faces.push(face.id);
            for loop_record in &mut face.loops {
                let mut rebuilt: Vec<CoedgeRecord> = Vec::with_capacity(loop_record.coedges.len());
                for coedge in &loop_record.coedges {
                    if !network.edges.contains(&coedge.edge_id) {
                        rebuilt.push(coedge.clone());
                        continue;
                    }
                    let Some(&strip_index) = network.rim_strip.get(&coedge.edge_id) else {
                        continue;
                    };
                    let rim = &edge_records[&coedge.edge_id];
                    let from = if coedge.forward { rim.start_vertex_id } else { rim.end_vertex_id };
                    let sharp = &sharp_edges[strip_index];
                    // The sharp edge runs from the strip's first junction.
                    let forward =
                        network.junction_of[&from] == network.strips[strip_index].junctions[0];
                    orientation.entry(sharp.id).or_default().push(forward);
                    rebuilt.push(CoedgeRecord {
                        id: alloc(),
                        edge_id: sharp.id,
                        forward,
                        // Placeholder; the re-trim/refit pass below recomputes it.
                        pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0))
                            .or_refuse(KernelStage::Sew, "make_line")?,
                    });
                }
                if rebuilt.is_empty() {
                    return Err(KernelRefusal::internal(
                        KernelStage::Sew,
                        "emptied_loop",
                        format!(
                            "{op}: healing the blend network emptied a loop of face {}",
                            face.id
                        ),
                    ));
                }
                loop_record.coedges = rebuilt;
            }
        }
    }

    // --- Close each gap: a split junction's section arc, or a standing cap's
    //     spokes (one from the apex to the junction's vertex, or two through it
    //     from one apex to another on a wall both lie on) ----------------------
    if !arcs.is_empty() || !spokes.is_empty() {
        let mut vertex_ends: HashMap<u64, (u64, u64)> = HashMap::default();
        for edge in solid.edges.iter().chain(&sharp_edges).chain(&arcs) {
            vertex_ends.insert(edge.id, (edge.start_vertex_id, edge.end_vertex_id));
        }
        let traversed = |coedge: &CoedgeRecord| -> Option<(u64, u64)> {
            let (start, end) = *vertex_ends.get(&coedge.edge_id)?;
            Some(if coedge.forward { (start, end) } else { (end, start) })
        };
        let joins = |edge: &EdgeRecord, from: u64, to: u64| {
            (edge.start_vertex_id == from && edge.end_vertex_id == to)
                || (edge.start_vertex_id == to && edge.end_vertex_id == from)
        };
        for shell in &mut solid.shells {
            for face in &mut shell.faces {
                if !touched_faces.contains(&face.id) {
                    continue;
                }
                let face_spokes: Vec<&EdgeRecord> = spokes
                    .iter()
                    .filter(|(_, walls)| walls.contains(&face.id))
                    .map(|(spoke, _)| spoke)
                    .collect();
                for loop_record in &mut face.loops {
                    let count = loop_record.coedges.len();
                    let mut closed: Vec<CoedgeRecord> = Vec::with_capacity(count + 2);
                    for position in 0..count {
                        let coedge = &loop_record.coedges[position];
                        closed.push(coedge.clone());
                        let next = &loop_record.coedges[(position + 1) % count];
                        let (Some((_, tail)), Some((head, _))) = (traversed(coedge), traversed(next))
                        else {
                            return Err(KernelRefusal::internal(
                                KernelStage::Sew,
                                "unknown_edge",
                                format!(
                                    "{op}: a healed loop of face {} uses an unknown edge",
                                    face.id
                                ),
                            ));
                        };
                        if tail == head {
                            continue;
                        }
                        let mut path: Vec<(&EdgeRecord, bool)> = Vec::new();
                        if let Some(arc) = arcs.iter().find(|arc| joins(arc, tail, head)) {
                            path.push((arc, arc.start_vertex_id == tail));
                        } else if let Some(spoke) = face_spokes.iter().find(|spoke| joins(spoke, tail, head)) {
                            path.push((spoke, spoke.start_vertex_id == tail));
                        } else {
                            // Apex to apex through the junction's vertex.
                            let first = face_spokes.iter().find(|spoke| spoke.start_vertex_id == tail);
                            let second = face_spokes.iter().find(|spoke| spoke.start_vertex_id == head);
                            if let (Some(first), Some(second)) = (first, second) {
                                if first.end_vertex_id == second.end_vertex_id {
                                    path.push((first, true));
                                    path.push((second, false));
                                }
                            }
                        }
                        if path.is_empty() {
                            return Err(KernelRefusal::internal(
                                KernelStage::Sew,
                                "loop_gap",
                                format!(
                                    "{op}: healing the blend network left a gap in a loop of face {}",
                                    face.id
                                ),
                            ));
                        }
                        for (edge, forward) in path {
                            orientation.entry(edge.id).or_default().push(forward);
                            closed.push(CoedgeRecord {
                                id: alloc(),
                                edge_id: edge.id,
                                forward,
                                // Placeholder; the re-trim/refit pass below recomputes it.
                                pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0))
                                    .or_refuse(KernelStage::Sew, "make_line")?,
                            });
                        }
                    }
                    loop_record.coedges = closed;
                }
            }
        }
    }
    for edge in sharp_edges
        .iter()
        .chain(&arcs)
        .chain(spokes.iter().map(|(spoke, _)| spoke))
    {
        let uses = orientation.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
        if uses.len() != 2 || uses[0] == uses[1] {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "edge_uses",
                format!(
                    "{op}: a recovered edge is not used once in each direction by its two \
                 faces (unexpected connectivity)"
                ),
            ));
        }
    }

    // --- Prune the network, its edges and its vertices -----------------------
    solid.shells[network.shell_index]
        .faces
        .retain(|face| !network.faces.contains(&face.id));
    solid.edges.retain(|edge| !network.edges.contains(&edge.id));
    // A held apex is its junction's vertex, and a standing cap's apex keeps its
    // cap: both stay.
    let removed_vertices: HashSet<u64> = network
        .vertices
        .iter()
        .copied()
        .filter(|vertex| !fixed_apexes.contains(vertex))
        .collect();
    solid
        .vertices
        .retain(|vertex| !removed_vertices.contains(&vertex.id));
    solid.edges.extend(sharp_edges.iter().cloned());
    solid.edges.extend(arcs.iter().cloned());
    solid.edges.extend(spokes.iter().map(|(spoke, _)| spoke.clone()));
    for end in &ends {
        match end {
            JunctionEnd::Point { vertex, .. } if fixed_apexes.contains(vertex) => {}
            JunctionEnd::Point { vertex, point } => solid.vertices.push(VertexRecord {
                id: *vertex,
                point: *point,
            }),
            JunctionEnd::Split {
                vertices, points, ..
            } => {
                for side in 0..2 {
                    solid.vertices.push(VertexRecord {
                        id: vertices[side],
                        point: points[side],
                    });
                }
            }
        }
    }

    // --- Re-trim and refit every plane the network touched -------------------
    let final_edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    // A plane whose loops wear neither a network edge nor a relocated one — a
    // standing cap, whose apex stays — is left exactly as it was.
    let mut retrim_order: Vec<u64> = network
        .planes
        .keys()
        .copied()
        .filter(|face_id| {
            touched_faces.contains(face_id)
                || find_face(&solid, *face_id).is_some_and(|(shell, position)| {
                    solid.shells[shell].faces[position]
                        .loops
                        .iter()
                        .flat_map(|loop_record| &loop_record.coedges)
                        .any(|coedge| relocated.contains(&coedge.edge_id))
                })
        })
        .collect();
    retrim_order.sort_unstable();
    let missing_face = |id: u64| {
        KernelRefusal::internal(KernelStage::Collect, "face", format!("{op}: missing face {id}"))
    };
    for face_id in retrim_order {
        let (ns, nf) = find_face(&solid, face_id).ok_or_else(|| missing_face(face_id))?;
        retrim_planar_face(
            &mut solid.shells[ns].faces[nf],
            &network.planes[&face_id],
            &final_edges,
            scale,
            op,
        )
        ?;
        // The planar re-trim maps each edge's WHOLE curve; a widened side edge
        // or an untouched subrange edge needs the range fitter.
        let touched: HashSet<u64> = solid.shells[ns].faces[nf]
            .loops
            .iter()
            .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
            .collect();
        refit_touched_pcurves(
            &mut solid.shells[ns].faces[nf],
            &final_edges,
            &touched,
            true,
            tolerance,
            op,
        )?;
    }
    // A kept strip keeps its carrier; every pcurve on it is refit to the edges
    // it now wears — its trimmed rims and its section arcs.
    let mut kept_order: Vec<u64> = network.kept.keys().copied().collect();
    kept_order.sort_unstable();
    for face_id in kept_order {
        let (ns, nf) = find_face(&solid, face_id).ok_or_else(|| missing_face(face_id))?;
        let touched: HashSet<u64> = solid.shells[ns].faces[nf]
            .loops
            .iter()
            .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
            .collect();
        refit_touched_pcurves(
            &mut solid.shells[ns].faces[nf],
            &final_edges,
            &touched,
            false,
            tolerance,
            op,
        )?;
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: blend network heal failed validation: {issues:?}"),
        ));
    }
    let chi_after = euler_characteristic(&solid);
    if chi_after != chi_before {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "euler",
            format!(
                "{op}: healing the blend network changed the Euler characteristic from \
             {chi_before} to {chi_after} — refusing rather than adding or removing a handle"
            ),
        ));
    }
    Ok(solid)
}
